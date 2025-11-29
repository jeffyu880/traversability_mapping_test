#include "utility.h"
#include "elevation_msgs/msg/occupancy_elevation.hpp"

// The file implements a probabilistic roadmap path planner

class TraversabilityPRM : public rclcpp::Node {
private:

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subGoal;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubPRMGraph; // publish PRM nodes and edges
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubPRMPath; // path extracted from roadmap
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubGlobalPath; // path is published in pose array format 
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubSingleSourcePaths; // publish paths to al states in roadmap

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudPRMNodes;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudPRMGraph;

    rclcpp::Subscription<elevation_msgs::msg::OccupancyElevation>::SharedPtr subElevationMap; // 2d local height map from mapping package

    elevation_msgs::msg::OccupancyElevation elevationMap; // this is received from mapping package. it is a 2d local map that includes height info

    float map_min[3]; // 0 - x, 1 - y, 2 - z
    float map_max[3];
 
    ///////////// Planner ////////////
    std::mutex mtx;

    bool costUpdateFlag[NUM_COSTS];

    nav_msgs::msg::Path displayGlobalPath; // path

    bool planningFlag; // set to "true" once goal is received from move_base

    state_t *robotState;
    state_t *goalState;
    state_t *mapCenter;

    kdtree_t* kdtree;

    vector<state_t*> nodeList; // roadmap nodeList
    
    vector<state_t*> pathList; // path through roadmap

public:
    TraversabilityPRM() : Node("traversability_prm"),
        planningFlag(false) {

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        robotState = new state_t;
        goalState = new state_t;
        mapCenter = new state_t;

        subGoal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/prm_goal", 5, std::bind(&TraversabilityPRM::goalPosHandler, this, std::placeholders::_1));
        
        subElevationMap = this->create_subscription<elevation_msgs::msg::OccupancyElevation>(
            "/occupancy_map_local_height", 5, std::bind(&TraversabilityPRM::elevationMapHandler, this, std::placeholders::_1));     

        pubPRMGraph = this->create_publisher<visualization_msgs::msg::MarkerArray>("/prm_graph", 5);
        pubPRMPath = this->create_publisher<visualization_msgs::msg::MarkerArray>("/prm_path", 5);
        pubSingleSourcePaths = this->create_publisher<visualization_msgs::msg::MarkerArray>("/prm_single_source_paths", 5);
        pubCloudPRMNodes = this->create_publisher<sensor_msgs::msg::PointCloud2>("/prm_cloud_nodes", 5);
        pubCloudPRMGraph = this->create_publisher<sensor_msgs::msg::PointCloud2>("/prm_cloud_graph", 5);

        pubGlobalPath = this->create_publisher<nav_msgs::msg::Path>("/global_path", 5);

        allocateMemory(); 
    }

    ~TraversabilityPRM(){}

    void allocateMemory(){
        kdtree = kd_create(3);

        for (int i = 0; i < NUM_COSTS; ++i)
            costUpdateFlag[i] = false;
        for (int i = 0; i < costHierarchy.size(); ++i)
            costUpdateFlag[costHierarchy[i]] = true;
    }

    void elevationMapHandler(const elevation_msgs::msg::OccupancyElevation::SharedPtr mapMsg){
        std::lock_guard<std::mutex> lock(mtx);

        elevationMap = *mapMsg;

        updateMapBoundary();

        updateCostMap();

        buildRoadMap();
    }

    void updateMapBoundary()
    {
        if (elevationMap.elevation.size() == 0)
            return;

        // Update map boundary based on occupancy grid info
        map_min[0] = elevationMap.occupancy.info.origin.position.x;
        map_min[1] = elevationMap.occupancy.info.origin.position.y;
        
        // Calculate map maximum bounds
        map_max[0] = elevationMap.occupancy.info.origin.position.x + elevationMap.occupancy.info.width * elevationMap.occupancy.info.resolution;
        map_max[1] = elevationMap.occupancy.info.origin.position.y + elevationMap.occupancy.info.height * elevationMap.occupancy.info.resolution;

        map_min[2] = -10; // z min
        map_max[2] = 10; // z max

        // Update map center for sampling
        mapCenter->x[0] = (map_min[0] + map_max[0]) / 2.0;
        mapCenter->x[1] = (map_min[1] + map_max[1]) / 2.0;
        mapCenter->x[2] = (map_min[2] + map_max[2]) / 2.0;
    }

    void updateCostMap(){
        // Cost map already provided through elevation map
        // Additional processing can be done here if needed
    }

    void buildRoadMap(){
        // 1. generate samples
        generateSamples();
        // 2. Add edges and update state height if map is changed
        updateStatesAndEdges();   
        // 3. Planning
        bfsSearch();
        // 4. Visualize Roadmap
        publishPRM();
        // 5. Publish path to move_base and stop planning
        publishPathStop();
        // 6. Convert PRM Graph into point cloud for external usage
        publishRoadmap2Cloud();
    }
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////// Planner /////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void goalPosHandler(const geometry_msgs::msg::PoseStamped::SharedPtr goal){
    	
        goalState->x[0] = goal->pose.position.x;
        goalState->x[1] = goal->pose.position.y;
        goalState->x[2] = goal->pose.position.z;
        
        // start planning
        planningFlag = true;
    }

    bool bfsSearch(){
        if (nodeList.size() == 0) 
            return false;

        getRobotState();

        // 1. Find the nearest node to robot (start node)
        state_t* startState = getNearestState(robotState);
        if (startState == NULL) return false;

        // 2. Find the nearest node to goal
        state_t* endState = getNearestState(goalState);
        if (endState == NULL) return false;

        // 3. Run BFS to find path
        for (auto state : nodeList) {
            state->cost = FLT_MAX;
            state->parentState = NULL;
        }

        queue<state_t*> stateQueue;
        startState->cost = 0;
        stateQueue.push(startState);

        while (!stateQueue.empty()) {
            state_t* currentState = stateQueue.front();
            stateQueue.pop();

            for (auto& neighbor : currentState->neighborList) {
                state_t* neighborState = neighbor.neighbor;
                float newCost = currentState->cost + neighbor.edgeCosts[0]; // use first cost type

                if (newCost < neighborState->cost) {
                    neighborState->cost = newCost;
                    neighborState->parentState = currentState;
                    stateQueue.push(neighborState);
                }
            }
        }

        // 4. Extract path
        pathList.clear();
        state_t* currentState = endState;
        while (currentState != NULL) {
            pathList.insert(pathList.begin(), currentState);
            currentState = currentState->parentState;
        }

        return pathList.size() > 0;
    }

    void generateSamples(){
        double sampling_start_time = rclcpp::Clock().now().seconds();
        while (rclcpp::Clock().now().seconds() - sampling_start_time < 0.002 && rclcpp::ok()){

            state_t* newState = new state_t;

            if (sampleState(newState)){
                // 1.1 Too close discard
                if (nodeList.size() != 0 && stateTooClose(newState) == true){
                    delete newState;
                    continue;
                }
                // 1.2 Save new state and insert to KD-tree
                newState->stateId = nodeList.size(); // mark state ID
                nodeList.push_back(newState);
                insertIntoKdtree(newState);
            }
            else
                delete newState;
        }
    }

    bool sampleState(state_t *stateCurr){
        // Sample within map boundaries
        stateCurr->x[0] = map_min[0] + (map_max[0] - map_min[0]) * ((double)rand() / RAND_MAX);
        stateCurr->x[1] = map_min[1] + (map_max[1] - map_min[1]) * ((double)rand() / RAND_MAX);
        stateCurr->x[2] = getStateHeight(stateCurr);

        // Check if state is valid (not in collision)
        if (isIncollision(stateCurr))
            return false;

        stateCurr->theta = 2 * M_PI * ((double)rand() / RAND_MAX);
        
        return true;
    }

    double getStateHeight(state_t* stateIn){
        // Get height from elevation map
        int x = (stateIn->x[0] - elevationMap.occupancy.info.origin.position.x) / elevationMap.occupancy.info.resolution;
        int y = (stateIn->x[1] - elevationMap.occupancy.info.origin.position.y) / elevationMap.occupancy.info.resolution;
        
        if (x >= 0 && x < elevationMap.occupancy.info.width && y >= 0 && y < elevationMap.occupancy.info.height) {
            int index = x + y * elevationMap.occupancy.info.width;
            if (index < elevationMap.elevation.size()) {
                return elevationMap.elevation[index];
            }
        }
        
        return 0.0; // default height
    }

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

    bool stateTooClose(state_t *stateIn){
        vector<state_t*> nearStates;
        getNearStates(stateIn, nearStates, neighborSampleRadius);
        return nearStates.size() > 0;
    }

    void updateStatesAndEdges(){
        // Update state heights and check edge validity
        for (auto state : nodeList) {
            state->x[2] = getStateHeight(state);
        }

        // Update edges between nearby states
        for (auto state : nodeList) {
            vector<state_t*> nearStates;
            getNearStates(state, nearStates, neighborConnectRadius);
            
            for (auto nearState : nearStates) {
                if (distance(state->x, nearState->x) < neighborConnectRadius) {
                    float edgeCosts[NUM_COSTS];
                    if (edgePropagation(state, nearState, edgeCosts)) {
                        // Add edge if not already exists
                        bool edgeExists = false;
                        for (auto& neighbor : state->neighborList) {
                            if (neighbor.neighbor == nearState) {
                                edgeExists = true;
                                break;
                            }
                        }
                        if (!edgeExists) {
                            neighbor_t newNeighbor;
                            newNeighbor.neighbor = nearState;
                            for (int i = 0; i < NUM_COSTS; ++i) {
                                newNeighbor.edgeCosts[i] = edgeCosts[i];
                            }
                            state->neighborList.push_back(newNeighbor);
                        }
                    }
                }
            }
        }
    }

    bool edgePropagation(state_t *state_from, state_t *state_to, float edgeCosts[NUM_COSTS]){
        // Simple distance cost for now
        float dist = distance(state_from->x, state_to->x);
        
        // Check if edge is valid (no collision)
        int steps = int(dist / 0.1); // check every 10cm
        for (int i = 0; i <= steps; ++i) {
            state_t testState;
            float ratio = float(i) / float(steps);
            testState.x[0] = state_from->x[0] + ratio * (state_to->x[0] - state_from->x[0]);
            testState.x[1] = state_from->x[1] + ratio * (state_to->x[1] - state_from->x[1]);
            testState.x[2] = state_from->x[2] + ratio * (state_to->x[2] - state_from->x[2]);
            
            if (isIncollision(&testState)) {
                return false;
            }
        }

        // Set edge costs
        edgeCosts[0] = dist; // distance cost
        edgeCosts[1] = abs(state_to->x[2] - state_from->x[2]); // elevation cost
        edgeCosts[2] = dist; // general cost

        return true;
    }

    void getNearStates(state_t *stateIn, vector<state_t*>& vectorNearStatesOut, double radius){
        kdres_t* result = kd_nearest_range(kdtree, stateIn->x, radius);
        
        while (!kd_res_end(result)) {
            state_t* nearState = (state_t*)kd_res_item_data(result);
            if (nearState != stateIn) {
                vectorNearStatesOut.push_back(nearState);
            }
            kd_res_next(result);
        }
        
        kd_res_free(result);
    }

    void insertIntoKdtree(state_t *stateCurr){
        kd_insert(kdtree, stateCurr->x, stateCurr);
    }

    state_t* getNearestState(state_t *stateIn){
        if (nodeList.empty()) return NULL;
        
        kdres_t* result = kd_nearest(kdtree, stateIn->x);
        if (kd_res_end(result)) {
            kd_res_free(result);
            return NULL;
        }
        
        state_t* nearestState = (state_t*)kd_res_item_data(result);
        kd_res_free(result);
        
        return nearestState;
    }

    float distance(double state_from[3], double state_to[3]){
        return sqrt((state_to[0]-state_from[0])*(state_to[0]-state_from[0]) + 
                    (state_to[1]-state_from[1])*(state_to[1]-state_from[1]) +
                    (state_to[2]-state_from[2])*(state_to[2]-state_from[2]));
    }

    void publishPRM(){        
        // Path visualization
        if (pubPRMPath->get_subscription_count() != 0){
            visualization_msgs::msg::MarkerArray markerArray;
            geometry_msgs::msg::Point p;

            // path visualization
            visualization_msgs::msg::Marker markerPath;
            markerPath.header.frame_id = "map";
            markerPath.header.stamp = this->get_clock()->now();
            markerPath.action = visualization_msgs::msg::Marker::ADD;
            markerPath.type = visualization_msgs::msg::Marker::LINE_STRIP;
            markerPath.ns = "path";
            markerPath.id = 0;
            markerPath.scale.x = 0.2;
            markerPath.color.r = 0.0; markerPath.color.g = 0; markerPath.color.b = 1.0;
            markerPath.color.a = 1.0;

            for (int i = 0; i < displayGlobalPath.poses.size(); ++i){
                p.x = displayGlobalPath.poses[i].pose.position.x;
                p.y = displayGlobalPath.poses[i].pose.position.y;
                p.z = displayGlobalPath.poses[i].pose.position.z;
                markerPath.points.push_back(p);
            }

            markerArray.markers.push_back(markerPath);

            // Node visualization
            visualization_msgs::msg::Marker markerNode;
            markerNode.header.frame_id = "map";
            markerNode.header.stamp = this->get_clock()->now();
            markerNode.action = visualization_msgs::msg::Marker::ADD;
            markerNode.type = visualization_msgs::msg::Marker::POINTS;
            markerNode.ns = "nodes";
            markerNode.id = 2;
            markerNode.scale.x = 0.2;
            markerNode.color.r = 0; markerNode.color.g = 1; markerNode.color.b = 1;
            markerNode.color.a = 1;

            for (int i = 0; i < nodeList.size(); ++i){
                if (distance(nodeList[i]->x, robotState->x) >= visualizationRadius)
                    continue;
                p.x = nodeList[i]->x[0];
                p.y = nodeList[i]->x[1];
                p.z = nodeList[i]->x[2] + 0.13;
                markerNode.points.push_back(p);
            }

            markerArray.markers.push_back(markerNode);

            pubPRMPath->publish(markerArray);
        }
    }

    void publishPathStop(){
        if (pathList.size() == 0) return;

        displayGlobalPath.poses.clear();
        displayGlobalPath.header.frame_id = "map";
        displayGlobalPath.header.stamp = this->get_clock()->now();

        for (auto state : pathList) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = this->get_clock()->now();
            pose.pose.position.x = state->x[0];
            pose.pose.position.y = state->x[1];
            pose.pose.position.z = state->x[2];
            
            // Set orientation based on path direction
            tf2::Quaternion q;
            q.setRPY(0, 0, state->theta);
            pose.pose.orientation = tf2::toMsg(q);
            
            displayGlobalPath.poses.push_back(pose);
        }

        pubGlobalPath->publish(displayGlobalPath);
    }

    void publishRoadmap2Cloud(){
        // Convert roadmap to point cloud for external usage
        if (pubCloudPRMNodes->get_subscription_count() == 0) return;
        
        pcl::PointCloud<PointType> nodeCloud;
        for (auto state : nodeList) {
            PointType p;
            p.x = state->x[0];
            p.y = state->x[1];
            p.z = state->x[2];
            p.intensity = state->cost;
            nodeCloud.points.push_back(p);
        }
        
        sensor_msgs::msg::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(nodeCloud, laserCloudTemp);
        laserCloudTemp.header.frame_id = "map";
        laserCloudTemp.header.stamp = this->get_clock()->now();
        pubCloudPRMNodes->publish(laserCloudTemp);
    }

    void getRobotState(){
        try{
            auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            
            robotState->x[0] = transform.transform.translation.x;
            robotState->x[1] = transform.transform.translation.y;
            robotState->x[2] = transform.transform.translation.z;

            tf2::Quaternion q;
            tf2::fromMsg(transform.transform.rotation, q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            robotState->theta = yaw + M_PI; // change from -PI~PI to 0~2*PI
        }
        catch (tf2::TransformException& ex){ 
            RCLCPP_ERROR(this->get_logger(), "Transform Failure: %s", ex.what());
            return; 
        }
    }
};

int main(int argc, char** argv){

    rclcpp::init(argc, argv);

    auto node = std::make_shared<TraversabilityPRM>();

    RCLCPP_INFO(node->get_logger(), "Traversability Planner Started.");

    rclcpp::spin(node);
    
    rclcpp::shutdown();
    
    return 0;
}
