#include "utility.h"
#include "elevation_msgs/msg/occupancy_elevation.hpp"

class TraversabilityPath : public rclcpp::Node {

public:

    std::mutex mtx;

    rclcpp::Subscription<elevation_msgs::msg::OccupancyElevation>::SharedPtr subElevationMap; // 2d local height map from mapping package
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subGoal;

    elevation_msgs::msg::OccupancyElevation elevationMap; // this is received from mapping package. it is a 2d local map that includes height info

    float map_min[3]; // 0 - x, 1 - y, 2 - z
    float map_max[3];

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPathCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPathLibraryValid;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPathLibraryOrigin;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubGlobalPath; // path is published in pose array format 

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    const int pathDepth = 4;

    bool planningFlag; // set to "true" once goal is received from move_base

    const float angularVelocityMax = 7.0 / 180.0 * M_PI;
    const float angularVelocityRes = 0.5 / 180.0 * M_PI;
    const float angularVelocityMax2 = 0.5 / 180.0 * M_PI;
    const float angularVelocityRes2 = 0.25 / 180.0 * M_PI;
    const float forwardVelocity = 0.1;
    const float deltaTime = 1;
    const int simTime = 30;

    int stateListSize;
    vector<state_t*> stateList;
    vector<state_t*> pathList;

    PointType goalPoint;
    nav_msgs::msg::Path globalPath;

    pcl::PointCloud<PointType>::Ptr pathCloudLocal;
    pcl::PointCloud<PointType>::Ptr pathCloudGlobal;
    pcl::PointCloud<PointType>::Ptr pathCloudValid;

    pcl::PointCloud<PointType>::Ptr pathCloud;

    pcl::KdTreeFLANN<PointType>::Ptr kdTreeFromCloud;

    state_t *rootState;
    state_t *goalState;

    TraversabilityPath() : Node("traversability_path"),
        planningFlag(false){

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pubGlobalPath = this->create_publisher<nav_msgs::msg::Path>("/global_path", 5);

        pubPathCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/path_trajectory", 5);
        pubPathLibraryValid = this->create_publisher<sensor_msgs::msg::PointCloud2>("/path_library_valid", 5);
        pubPathLibraryOrigin = this->create_publisher<sensor_msgs::msg::PointCloud2>("/path_library_origin", 5);

        subGoal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/prm_goal", 5, std::bind(&TraversabilityPath::goalPosHandler, this, std::placeholders::_1));
        
        subElevationMap = this->create_subscription<elevation_msgs::msg::OccupancyElevation>(
            "/occupancy_map_local_height", 5, std::bind(&TraversabilityPath::elevationMapHandler, this, std::placeholders::_1));

        pathCloud.reset(new pcl::PointCloud<PointType>());
        pathCloudLocal.reset(new pcl::PointCloud<PointType>());
        pathCloudGlobal.reset(new pcl::PointCloud<PointType>());
        pathCloudValid.reset(new pcl::PointCloud<PointType>());

        kdTreeFromCloud.reset(new pcl::KdTreeFLANN<PointType>());

        rootState = new state_t;
        goalState = new state_t;

        createPathLibrary();
    }

    void createPathLibrary(){
        pathCloudLocal->clear();
        stateList.clear();

        rootState->x[0] = 0;
        rootState->x[1] = 0;
        rootState->x[2] = 0;
        rootState->theta = 0;

        createPathLibrary(rootState, 0);

        pathCloudLocal->width = pathCloudLocal->points.size();
        pathCloudLocal->height = 1;
        pathCloudLocal->is_dense = false;

        stateListSize = stateList.size();

        RCLCPP_INFO(this->get_logger(), "Path library created with %d states", stateListSize);
    }

    void createPathLibrary(state_t* parentState, int previousDepth){

        int thisDepth = previousDepth + 1;
        if (thisDepth > pathDepth)
            return;

        float currentAngVelMax = angularVelocityMax;
        float currentAngVelRes = angularVelocityRes;

        if (thisDepth >= 2){
            currentAngVelMax = angularVelocityMax2;
            currentAngVelRes = angularVelocityRes2;
        }

        for (float vTheta = -currentAngVelMax; vTheta <= currentAngVelMax + 1e-6; vTheta += currentAngVelRes){

            state_t *previousState = parentState;

            for (int i = 0; i < simTime; ++i){

                state_t *newState = new state_t;

                newState->x[0] = previousState->x[0] + (forwardVelocity * cos(previousState->theta)) * deltaTime;
                newState->x[1] = previousState->x[1] + (forwardVelocity * sin(previousState->theta)) * deltaTime;
                newState->x[2] = 0;
                newState->theta = previousState->theta + vTheta * deltaTime;

                newState->stateId = stateList.size();

                PointType p;
                p.x = newState->x[0];
                p.y = newState->x[1];
                p.z = newState->x[2];
                p.intensity = newState->stateId;

                pathCloudLocal->push_back(p);
                stateList.push_back(newState);

                previousState = newState;
            }

            createPathLibrary(previousState, thisDepth);
        }
    }

    void elevationMapHandler(const elevation_msgs::msg::OccupancyElevation::SharedPtr mapMsg){

        std::lock_guard<std::mutex> lock(mtx);

        elevationMap = *mapMsg;

        updateCostMap();

        updatePathLibrary();

        if (planningFlag == false)
            return;

        updateTrajectory();

        publishTrajectory();
    }

    void goalPosHandler(const geometry_msgs::msg::PoseStamped::SharedPtr goal){
        
        goalPoint.x = goal->pose.position.x;
        goalPoint.y = goal->pose.position.y;
        goalPoint.z = goal->pose.position.z;

        goalState->x[0] = goalPoint.x;
        goalState->x[1] = goalPoint.y;
        goalState->x[2] = goalPoint.z;
        
        // start planning
        planningFlag = true;
    }

    void updateCostMap()
    {
        if (elevationMap.elevation.size() == 0)
            return;

        // Update map boundary based on occupancy grid info
        map_min[0] = elevationMap.occupancy.info.origin.position.x;
        map_min[1] = elevationMap.occupancy.info.origin.position.y;
        
        // Calculate map maximum bounds
        map_max[0] = elevationMap.occupancy.info.origin.position.x + elevationMap.occupancy.info.width * elevationMap.occupancy.info.resolution;
        map_max[1] = elevationMap.occupancy.info.origin.position.y + elevationMap.occupancy.info.height * elevationMap.occupancy.info.resolution;
    }

    void updatePathLibrary(){

        // 1. Reset valid flag
        for (int i = 0; i < stateListSize; ++i){
            stateList[i]->validFlag = true;
        }

        // 2. Transform local paths to global paths
        try{
            auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

            // Transform the point cloud
            sensor_msgs::msg::PointCloud2 cloud_in_msg, cloud_out_msg;
            pcl::toROSMsg(*pathCloudLocal, cloud_in_msg);
            cloud_in_msg.header.frame_id = "base_link";
            cloud_in_msg.header.stamp = this->get_clock()->now();

            tf2::doTransform(cloud_in_msg, cloud_out_msg, transform);
            pcl::fromROSMsg(cloud_out_msg, *pathCloudGlobal);

        }
        catch (tf2::TransformException& ex){ 
            RCLCPP_ERROR(this->get_logger(), "Transform Failure: %s", ex.what());
            return; 
        }

        // 3. Collision check
        state_t *state = new state_t;
        for (int i = 0; i < stateListSize; ++i){
            
            PointType *p = &pathCloudGlobal->points[i];
            
            updateHeight(p);

            state->x[0] = p->x;
            state->x[1] = p->y;
            state->x[2] = p->z;

            if (isIncollision(state) == true){
                markInvalidState(stateList[i]);
                continue;
            }
        }

        delete state;

        // 4. Save valid paths
        pathCloudValid->clear();
        for (int i = 0; i < stateListSize; ++i){
            if (stateList[i]->validFlag == true){
                PointType p = pathCloudGlobal->points[i];
                p.intensity = 0;
                pathCloudValid->push_back(p);
            }
        }

        // 5. Visualize valid library states (or paths)
        if (pubPathLibraryValid->get_subscription_count() != 0){
            sensor_msgs::msg::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*pathCloudValid, laserCloudTemp);
            laserCloudTemp.header.stamp = this->get_clock()->now();
            laserCloudTemp.header.frame_id = "map";
            pubPathLibraryValid->publish(laserCloudTemp);
        }

        // 6. Visualize all library states (or paths)
        if (pubPathLibraryOrigin->get_subscription_count() != 0){
            sensor_msgs::msg::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*pathCloudLocal, laserCloudTemp);
            laserCloudTemp.header.stamp = this->get_clock()->now();
            laserCloudTemp.header.frame_id = "base_link";
            pubPathLibraryOrigin->publish(laserCloudTemp);
        }
    }

    void updateHeight(PointType *p){
        // Get height from elevation map
        int x = (p->x - elevationMap.occupancy.info.origin.position.x) / elevationMap.occupancy.info.resolution;
        int y = (p->y - elevationMap.occupancy.info.origin.position.y) / elevationMap.occupancy.info.resolution;
        
        if (x >= 0 && x < elevationMap.occupancy.info.width && y >= 0 && y < elevationMap.occupancy.info.height) {
            int index = x + y * elevationMap.occupancy.info.width;
            if (index < elevationMap.elevation.size()) {
                p->z = elevationMap.elevation[index];
            }
        }
    }

    void updateTrajectory(){
        // Use KD-tree to find best path toward goal
        kdTreeFromCloud->setInputCloud(pathCloudValid);
        
        vector<int> pointIdxSearch;
        vector<float> pointSquaredDistance;

        PointType goalPt = goalPoint;
        
        int numberOfPointsFound = kdTreeFromCloud->nearestKSearch(goalPt, 1, pointIdxSearch, pointSquaredDistance);
        
        pathList.clear();
        
        if (numberOfPointsFound > 0) {
            int bestStateId = pathCloudValid->points[pointIdxSearch[0]].intensity;
            
            // Trace back through path library to get full trajectory
            for (int i = 0; i < stateListSize; ++i) {
                if (stateList[i]->stateId == bestStateId && stateList[i]->validFlag) {
                    pathList.push_back(stateList[i]);
                    break;
                }
            }
        }
    }

    void publishTrajectory(){

        if (pubPathCloud->get_subscription_count() != 0){
            pathCloud->clear();
            for (int i = 0; i < pathList.size(); ++i){
                PointType p = pathCloudGlobal->points[pathList[i]->stateId];
                p.z += 0.1;
                p.intensity = pathList[i]->cost;
                pathCloud->push_back(p);
            }
            sensor_msgs::msg::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*pathCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = this->get_clock()->now();
            laserCloudTemp.header.frame_id = "map";
            pubPathCloud->publish(laserCloudTemp);
        }

        // even no feasible path is found, publish an empty path
        globalPath.poses.clear();
        for (int i = 0; i < pathList.size(); i++){
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = this->get_clock()->now();
            pose.pose.position.x = pathList[i]->x[0];
            pose.pose.position.y = pathList[i]->x[1];
            pose.pose.position.z = pathList[i]->x[2];

            // Set orientation
            tf2::Quaternion q;
            q.setRPY(0, 0, pathList[i]->theta);
            pose.pose.orientation = tf2::toMsg(q);

            globalPath.poses.push_back(pose);
        }

        globalPath.header.frame_id = "map";
        globalPath.header.stamp = this->get_clock()->now();
        pubGlobalPath->publish(globalPath);
    }

    void markInvalidState(state_t* state){
        state->validFlag = false;
        
        // Mark all child states as invalid too
        for (auto child : state->childList) {
            if (child->validFlag) {
                markInvalidState(child);
            }
        }
    }

    // Collision check (using state for input)
    bool isIncollision(state_t* stateIn){
        // Check collision using elevation map
        int x = (stateIn->x[0] - elevationMap.occupancy.info.origin.position.x) / elevationMap.occupancy.info.resolution;
        int y = (stateIn->x[1] - elevationMap.occupancy.info.origin.position.y) / elevationMap.occupancy.info.resolution;
        
        if (x >= 0 && x < elevationMap.occupancy.info.width && y >= 0 && y < elevationMap.occupancy.info.height) {
            int index = x + y * elevationMap.occupancy.info.width;
            if (index < elevationMap.occupancy.data.size()) {
                return elevationMap.occupancy.data[index] > 50; // occupied if > 50%
            }
        }
        
        return true; // collision if out of bounds
    }

    float distance(double state_from[3], double state_to[3]){
        return sqrt((state_to[0]-state_from[0])*(state_to[0]-state_from[0]) + 
                    (state_to[1]-state_from[1])*(state_to[1]-state_from[1]) +
                    (state_to[2]-state_from[2])*(state_to[2]-state_from[2]));
    }

    void publishPath(){
        sensor_msgs::msg::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*pathCloudValid, laserCloudTemp);
        laserCloudTemp.header.stamp = this->get_clock()->now();
        laserCloudTemp.header.frame_id = "base_link";
        pubPathLibraryValid->publish(laserCloudTemp);
    }
};

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TraversabilityPath>();

    RCLCPP_INFO(node->get_logger(), "Traversability Planner Started.");

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}