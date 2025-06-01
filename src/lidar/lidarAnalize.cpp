#include "lidar/lidarAnalize.h"
#include "defs/constante.h" // DEG_TO_RAD
#include <math.h>


void convertAngularToAxial(lidarAnalize_t* data, int count, position_t position, int narrow){
    for(int i = 0; i< count; i++){
        data[i].x =  data[i].dist * cos(double((data[i].angle - position.a) * DEG_TO_RAD)) + position.x;
        data[i].y = -data[i].dist * sin(double((data[i].angle - position.a) * DEG_TO_RAD)) + position.y;
        
        data[i].onTable = (data[i].x <  1000 - narrow && 
                           data[i].x > -1000 + narrow && 
                           data[i].y <  1500 - narrow && 
                           data[i].y > -1500 + narrow);
    }
}

double average_angles(double angles[], int count) {
    double x_sum = 0.0, y_sum = 0.0;
    
    for (int i = 0; i < count; i++) {
        // Convert angle to unit vector
        x_sum += cos(angles[i] * DEG_TO_RAD);
        y_sum += sin(angles[i] * DEG_TO_RAD);
    }

    // Calculate the average vector
    double avg_x = x_sum / count;
    double avg_y = y_sum / count;

    // Convert back to angle
    double avg_angle = atan2(avg_y, avg_x) * RAD_TO_DEG;

    // Normalize angle to [0, 360) if needed
    if (avg_angle < 0) {
        avg_angle += 360.0;
    }

    return avg_angle;
}

double distance_2_pts(double d1,double deg1, double d2, double deg2){
    double alpha;
    double d3;
    alpha = fabs((deg1-deg2)*M_PI/180);
    d3 = sqrt(pow(d1,2) + pow(d2,2) -2*d1*d2*cos(alpha));
    return d3;
}

#define BLOBS_COUNT 200 // Define the maximum number of blobs

typedef struct {
    int index_start;
    int index_stop;
    int count;
} lidar_blob_detection;

//Calculate the average diameter of a blob, also returns the dist 
void blob_delimiter(lidarAnalize_t* data, int count, lidar_blob_detection blob, double* diam, double* dist, double* angle){
    //Using the min and max angles seen by the lidar of the blob, you can find a circle the circles the blob

    double sum_r = 0;
    if (blob.index_stop >= blob.index_start){
        for (int i = blob.index_start; i <= blob.index_stop; i++) sum_r += data[i].dist;
    }
    else{
        // edgecase where blob right in front
        for (int i = blob.index_start; i < count; i++) sum_r += data[i].dist;
        for (int i = 0; i <= blob.index_stop; i++) sum_r += data[i].dist;
    }

    *dist = sum_r / (double)blob.count;
    double angles[] = {data[blob.index_start].angle, data[blob.index_stop]. angle};
    *angle = average_angles(angles, 2);

    double distance1 = distance_2_pts(*dist, *angle, data[blob.index_start].dist, angles[0]);
    double distance2 = distance_2_pts(*dist, *angle, data[blob.index_stop].dist, angles[1]);

    *diam = 2 * (distance1 > distance2 ? distance1 : distance2);    
}

// Returns the number of blobs found
int find_blobs(lidarAnalize_t* data, int count, lidar_blob_detection* blobs, int min_points, int max_distance){
    //LOG_DEBUG("Total lidar point count is ", count, ", minium to be a blob is ", min_points);
    
    int blob_idx = 0; // Count of detected blobs

    // Initialize blobs
    for (int b = 0; b < BLOBS_COUNT; b++) {
        blobs[b].index_start = 0;
        blobs[b].index_stop = 0;
        blobs[b].count = 0;
    }

    // Iterate through points and create blobs
    for (int i = 0; i < count; i++) {

        if (!data[i].onTable){
            if (blobs[blob_idx].count > min_points){
                blob_idx++;
                if (blob_idx == BLOBS_COUNT){
                    LOG_WARNING("position_opponentV2 - No more place for more blobs!");
                    return -1;
                }
            }
            else if (blobs[blob_idx].count > 0){
                blobs[blob_idx].count = 0;
            }
            continue;
        }

        if (blobs[blob_idx].count == 0){
            blobs[blob_idx].index_start = i;
            blobs[blob_idx].index_stop = i;
            blobs[blob_idx].count = 1;
        }
        else if (fabs(data[blobs[blob_idx].index_stop].dist - data[i].dist) > max_distance || data[i].angle - data[blobs[blob_idx].index_stop].angle > 1.0){ //If two points are further than max_distance then create a new blob
            if (blobs[blob_idx].count >= min_points)
                blob_idx++;
            if (blob_idx == BLOBS_COUNT){
                LOG_WARNING("position_opponentV2 - No more place for more blobs!");
                return -1;
            }
            blobs[blob_idx].index_start = i;
            blobs[blob_idx].index_stop = i;
            blobs[blob_idx].count = 1;
        }
        else{ //Add the point to the current blob
            blobs[blob_idx].index_stop = i;
            blobs[blob_idx].count ++;
        }
    }
    //Check for the edge case where opponent is right in front
    if (data[0].onTable && data[count - 1].onTable && fabs(data[0].dist - data[count - 1].dist) < max_distance){
        //Merge the last into the first
        blobs[0].index_start = blobs[blob_idx].index_start;
        blobs[0].count += blobs[blob_idx].count;
    }
    return blob_idx;
}

// Dans le pire des cas, on a 0.3% des points qui sont l'ennemi
bool position_opponentV2(lidarAnalize_t* data, int count, position_t robot_pos, position_t *opponent_pos){
    lidar_blob_detection blobs[BLOBS_COUNT]; // Array to hold detected blobs
    int min_points = 4 ; //4 points to be an opponent
    int max_distance = 50; //50mm to be a different blob
    int blob_count = find_blobs(data, count, blobs, min_points, max_distance);


    if (blob_count < 1){
        // LOG_DEBUG("Did not find any opponent blob");
        return false;
    }

    int opponent_idx = -1;

    // determination of the linearity of the blobs
    for (int j = 0; j < blob_count; j++) {
        lidar_blob_detection blob = blobs[j];

        // determination of the blob size
        double diam, dist, angle;
        blob_delimiter(data, count, blob, &diam, &dist, &angle);
        //LOG_DEBUG("Found a blob with diam=", diam, "mm, with ", blob.count, " points at ", angle, " degrees ", dist, "mm far");
        if (diam > 100 || diam < 10) continue; //TODO Min should be 50mm
        //LOG_GREEN_INFO("Blob fits the requirements");

        if (opponent_idx != -1){
            // LOG_WARNING("Too many opponents blobs were found, stopping");
            return false;
        }
        // Blob considered as opponent 
        opponent_idx = j;
    }

    if (opponent_idx == -1){
        return false;
    }

    // determination of the opponent could be using the bounding box of the blob
    lidar_blob_detection opponent_blob = blobs[opponent_idx];
    // Calculate the centroid of the largest blob
    int pos_sum_x = 0;
    int pos_sum_y = 0;
    if (opponent_blob.index_stop >= opponent_blob.index_start){
        for (int i = opponent_blob.index_start; i <= opponent_blob.index_stop; i++){
            pos_sum_x += data[i].x;
            pos_sum_y += data[i].y;
        }
    }
    else{
        // edgecase
        for (int i = opponent_blob.index_start; i < count; i++){
            pos_sum_x += data[i].x;
            pos_sum_y += data[i].y;
        }
        for (int i = 0; i <= opponent_blob.index_stop; i++){
            pos_sum_x += data[i].x;
            pos_sum_y += data[i].y;
        }
    }
    opponent_pos->x = pos_sum_x / opponent_blob.count;
    opponent_pos->y = pos_sum_y / opponent_blob.count;

    double angle_robot_opponent = atan2(opponent_pos->y - robot_pos.y, opponent_pos->x - robot_pos.x);
    opponent_pos->x += 15*cos(angle_robot_opponent); //Offset the position my 15mm
    opponent_pos->y += 15*sin(angle_robot_opponent);

    return true;
}

// width is robot's width
// depth is brake distance
// margin is the margin around radius a lidar point
bool opponent_collide_lidar(lidarAnalize_t* data, int count, int width, int depth, int margin){
    for (int i = 0; i < count; i++){
        if (!data[i].onTable) continue;
        // Check if the circle made by the lidar point and radius margin
        // overlaps with the rectangle made by width and depth
        int x = data[i].dist * cos(data[i].angle * DEG_TO_RAD); // front
        int y = -data[i].dist * sin(data[i].angle * DEG_TO_RAD); // sides
        // skip points that are outside the robot's depth
        if (x * depth < 0) continue;

        if (x + margin >= -abs(depth) && 
            x - margin <= abs(depth) && 
            y + margin >= -width/2 && 
            y - margin <= width/2){
            return true;
        }
    }
    return false;
}