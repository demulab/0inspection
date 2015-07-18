#ifndef __NAV_PARAM_H
#define __NAV_PARAM_H

const double DIST_LIMIT = 1.0;

// Waypoint
// 角度777が終了の合図 
                                                                            
const int wp_total_num = 3;
const double ox  = -0.683; // offset_x 
const double oy  = -1.973; // offset_y

enum  Robot_State { START_STATE, WP1_STATE, WP2_STATE, WP3_STATE, WP3_AGAIN_STATE,
		    WP4_STATE, NAVI_STATE, MOVE_PERSON_STATE, MOVE_OBJECT_STATE,
		    FOLLOW_STATE, GOAL_STATE };

                                                             
const double waypoint[wp_total_num+2][3] = {
  // x, y, theta               
  {0, 0, 0}, // dummy ウェイポイントの番号と配列の番号を同じにするため
  { 10.70, 3.00,  1.342}, // WP1  No.2
  {  2.5, 6.24,  2.876},  
  {0, 0, 777} // dummy 終了用}
} ;


#endif
