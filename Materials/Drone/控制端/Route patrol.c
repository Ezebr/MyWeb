#define MAP_WIDTH   9
#define MAP_HEIGHT  7
#define MAX_PATH    120

// 方向：上、下、左、右
static  int  DX[4]   = {-1, 1, 0, 0};
static  int  DY[4]   = { 0, 0,-1, 1};
static  char DIRC[4] = {'U','D','L','R'};  // 内部用字符
// 对外编码：右=1，左=2，前=3，后=4
static inline int dirchar_to_code(char d) {
    if (d == 'U') return 1;
    if (d == 'D') return 2;
    if (d == 'L') return 3;
    if (d == 'R') return 4;
    return 0;
}

static inline bool in_bounds(int r, int c) {
    return r >= 0 && r < MAP_HEIGHT && c >= 0 && c < MAP_WIDTH;
}

static inline bool passable(int r, int c, const int m[MAP_HEIGHT][MAP_WIDTH]) {
    return in_bounds(r,c) && (m[r][c] == 0);
}

// 校验三点是否连续（同行三连或同列三连）
static bool three_points_are_consecutive(const int stop[3][2]) {
    int r0 = stop[0][0], c0 = stop[0][1];
    int r1 = stop[1][0], c1 = stop[1][1];
    int r2 = stop[2][0], c2 = stop[2][1];

    // 同行
    if (r0 == r1 && r1 == r2) {
        int x = c0, y = c1, z = c2;
        if (x > y) { int t = x; x = y; y = t; }
        if (y > z) { int t = y; y = z; z = t; }
        if (x > y) { int t = x; x = y; y = t; }
        return (x + 1 == y) && (y + 1 == z);
    }
    // 同列
    if (c0 == c1 && c1 == c2) {
        int x = r0, y = r1, z = r2;
        if (x > y) { int t = x; x = y; y = t; }
        if (y > z) { int t = y; y = z; z = t; }
        if (x > y) { int t = x; x = y; y = t; }
        return (x + 1 == y) && (y + 1 == z);
    }
    return false;
}

// 最短路 BFS：从 (sr,sc) 到 (tr,tc)，把 U/D/L/R 写入 out_dir，返回步数；不可达返回 -1
static int bfs_shortest(const int m[MAP_HEIGHT][MAP_WIDTH],
                        int sr, int sc, int tr, int tc, char *out_dir)
{
    if (sr == tr && sc == tc) return 0;

    static int  qx[MAP_HEIGHT * MAP_WIDTH];
    static int  qy[MAP_HEIGHT * MAP_WIDTH];
    static bool vis[MAP_HEIGHT][MAP_WIDTH];
    static char pre_dir[MAP_HEIGHT][MAP_WIDTH];

    memset(vis, 0, sizeof(vis));
    memset(pre_dir, 0, sizeof(pre_dir));

    int front = 0, rear = 0;
    qx[rear] = sr; qy[rear] = sc; rear++;
    vis[sr][sc] = true;

    bool found = false;
    while (front < rear) {
        int r = qx[front], c = qy[front]; front++;
        for (int k = 0; k < 4; ++k) {
            int nr = r + DX[k], nc = c + DY[k];
            if (!passable(nr,nc,m) || vis[nr][nc]) continue;
            vis[nr][nc]  = true;
            pre_dir[nr][nc] = DIRC[k];
            qx[rear] = nr; qy[rear] = nc; rear++;
            if (nr == tr && nc == tc) { found = true; goto DONE; }
        }
    }
DONE:
    if (!found) return -1;

    // 回溯并反转
    char tmp[MAP_HEIGHT * MAP_WIDTH];
    int len = 0;
    int cr = tr, cc = tc;
    while (!(cr == sr && cc == sc)) {
        char d = pre_dir[cr][cc];
        tmp[len++] = d;
        if      (d == 'U') cr += 1;
        else if (d == 'D') cr -= 1;
        else if (d == 'L') cc += 1;
        else if (d == 'R') cc -= 1;
    }
    for (int i = 0; i < len; ++i) out_dir[i] = tmp[len - 1 - i];
    return len;
}

// 生成蛇形覆盖目标（自底向上，交替左右方向），仅包含可走格
static int build_snake_targets(const int m[MAP_HEIGHT][MAP_WIDTH],
                               int tr[], int tc[])
{
    int cnt = 0;
    for (int r = MAP_HEIGHT - 1, layer = 0; r >= 0; --r, ++layer) {
        if (layer % 2 == 0) { // 右->左
            for (int c = MAP_WIDTH - 1; c >= 0; --c)
                if (m[r][c] == 0) { tr[cnt] = r; tc[cnt] = c; cnt++; }
        } else {              // 左->右
            for (int c = 0; c < MAP_WIDTH; ++c)
                if (m[r][c] == 0) { tr[cnt] = r; tc[cnt] = c; cnt++; }
        }
    }
    return cnt;
}

/**
 * @brief  生成覆盖全图并回起点的路径（起点：右下角）。
 * @param  stop[3][2]  三个连续障碍（0 基坐标），每行为 {row, col}
 * @param  out_steps   输出的路径数组（上=1，下=2，左=3，右=4）
 * @param  max_steps   out_steps 的最大容量
 * @return 路径步数（>=0）；输入不合法返回 -1
 */
int build_cover_path_with_return(const int stop[3][2], int *out_steps, int max_steps)
{
    // 校验输入
    for (int i = 0; i < 3; ++i) {
        if (!in_bounds(stop[i][0], stop[i][1])) return -1;
    }
   // if (!three_points_are_consecutive(stop)) return -1;

    // 构建地图：0 可走；1 障碍
    int map[MAP_HEIGHT][MAP_WIDTH];
    memset(map, 0, sizeof(map));
    for (int i = 0; i < 3; ++i) {
        map[stop[i][0]][stop[i][1]] = 1;
    }

    // 起点（右下角）保证可走
    const int SR = MAP_HEIGHT - 1;
    const int SC = MAP_WIDTH  - 1;
    map[SR][SC] = 0; 

    // 生成蛇形目标序列，并找到起点在序列中的位置
    int tr[MAP_HEIGHT * MAP_WIDTH];
    int tc[MAP_HEIGHT * MAP_WIDTH];
    int tcnt = build_snake_targets(map, tr, tc);

    int sidx = -1;
    for (int i = 0; i < tcnt; ++i) {
        if (tr[i] == SR && tc[i] == SC) { sidx = i; break; }
    }
    if (sidx < 0) return 0; // 理论上不会发生

    int total_len = 0;
    int cr = SR, cc = SC;
    char seg[MAX_PATH];

    // 访问 sidx 之后的目标
    for (int i = sidx + 1; i < tcnt; ++i) {
        int len = bfs_shortest(map, cr, cc, tr[i], tc[i], seg);
        if (len < 0) continue;
        if (total_len + len > max_steps) len = max_steps - total_len;
        for (int k = 0; k < len; ++k) out_steps[total_len + k] = dirchar_to_code(seg[k]);
        total_len += len;
        if (total_len >= max_steps) return total_len;
        cr = tr[i]; cc = tc[i];
    }
    // 访问 sidx 之前的目标
    for (int i = 0; i < sidx; ++i) {
        int len = bfs_shortest(map, cr, cc, tr[i], tc[i], seg);
        if (len < 0) continue;
        if (total_len + len > max_steps) len = max_steps - total_len;
        for (int k = 0; k < len; ++k) out_steps[total_len + k] = dirchar_to_code(seg[k]);
        total_len += len;
        if (total_len >= max_steps) return total_len;
        cr = tr[i]; cc = tc[i];
    }

    // 最后回到起点
    {
        int len = bfs_shortest(map, cr, cc, SR, SC, seg);
        if (len > 0) {
            if (total_len + len > max_steps) len = max_steps - total_len;
            for (int k = 0; k < len; ++k) out_steps[total_len + k] = dirchar_to_code(seg[k]);
            total_len += len;
        }
    }

    return total_len; // 实际路径步数
}


int count =0;
int step1 = 0;
#define Waypoint_Fix_Cnt1   2 //5
#define Waypoint_Fix_CM1    10//5
//位置kp=400
int count_x=1;
int count_y=9;
int xcount=0;
static uint16_t openmv_work_mode=0x01;
// int stop[3][2] = { {3,4}, {3,5}, {3,6} };
int steps[MAX_PATH];

void Agriculture_UAV_Closeloop(void)
{	
	static uint8_t n=10;
	Vector3f target_position;
  int stop[3][2] = { {6,7}, {6,6}, {6,5} };
	if(count ==0)
	{
		if(stop[0][0]==0)
		{
		xcount++;
		}
		if(stop[1][0]==0)
		{
			xcount++;
		}
    if(stop[2][0]==0)
    {
			xcount++;
		}
     
		if(stop[0][1]==0||stop[1][1]==0||stop[2][1]==0)
		{
		xcount=xcount ;
		}
		else
		{
		xcount=0;
		}
		
		step1 = build_cover_path_with_return(stop, steps, MAX_PATH);//确定长度
	  count ++;
	}
	if(flight_subtask_cnt[n]==0)//起飞点作为第一个悬停点
	{
		basic_auto_flight_support();//基本飞行支持软件
		work_waypoint_generate();
		//记录下初始起点位置，实际项目中可设置为某一基准原点
		//base_position.x=VIO_SINS.Position[_EAST];
		//base_position.y=VIO_SINS.Position[_NORTH];
		base_position.z=NamelessQuad.Position[_UP];
		
		//execute_time_ms[n]=work_time_gap[0]/flight_subtask_delta;//子任务执行时间
		target_position.x=base_position.x+work_waypoints[0][0];
		target_position.y=base_position.y+work_waypoints[1][0];
		target_position.z=base_position.z;
		Horizontal_Navigation(target_position.x,
													target_position.y,
													target_position.z,
													GLOBAL_MODE,
													MAP_FRAME);
		flight_subtask_cnt[n]=11;
		flight_global_cnt[n]=0;
	}
	
	else if(flight_subtask_cnt[n]==11)
	{
	basic_auto_flight_support();//基本飞行支持软件
		if(flight_global_cnt[n]<Waypoint_Fix_Cnt1)//持续4*5ms=0.05s满足
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM1)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
	else
  	{
		target_position.x=base_position.x+5;
		target_position.y=base_position.y+5;
		target_position.z=base_position.z-5;
		Horizontal_Navigation(target_position.x,
													target_position.y,
													target_position.z,
													GLOBAL_MODE,
													MAP_FRAME);
			
			
		flight_subtask_cnt[n]=5;
		flight_global_cnt[n]=0;
		}
	}

		//向右移动50cm
	else if(flight_subtask_cnt[n]==1)
	{
		basic_auto_flight_support();//基本飞行支持软件
		SDK_DT_Send_Check(openmv_work_mode,UART3_SDK);
		if(flight_global_cnt[n]<Waypoint_Fix_Cnt1)//持续4*5ms=0.05s满足
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM1)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
	else
  	{
			Horizontal_Navigation(58,0,0,RELATIVE_MODE,BODY_FRAME);	
		  NCLink_Send_To_Firetruck(count_x,count_y,Opv_Top_View_Target.angle1,Opv_Top_View_Target.angle2,
			                       Opv_Top_View_Target.distance1, Opv_Top_View_Target.distance2,
			                       Opv_Top_View_Target.apriltag_id1,Opv_Top_View_Target.apriltag_id2,
			                       Opv_Top_View_Target.width1,Opv_Top_View_Target.width2,
			                       Opv_Top_View_Target.height1,Opv_Top_View_Target.height2);
      count_x++;			
      flight_subtask_cnt[n]=5;
      flight_global_cnt[n]=0;			
	}
}
	//向左移动50
		else if(flight_subtask_cnt[n]==2)
	{
		basic_auto_flight_support();//基本飞行支持软件
		ncq_control_althold();
		SDK_DT_Send_Check(openmv_work_mode,UART3_SDK);
			if(flight_global_cnt[n]<Waypoint_Fix_Cnt1)//持续4*5ms=0.05s满足
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM1)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
	else
  	{
			//向左移动50cm
		Horizontal_Navigation(-58,0,0,RELATIVE_MODE,BODY_FRAME);	
		NCLink_Send_To_Firetruck(count_x,count_y,Opv_Top_View_Target.angle1,Opv_Top_View_Target.angle2,
			                      Opv_Top_View_Target.distance1,Opv_Top_View_Target.distance2,
			                      Opv_Top_View_Target.apriltag_id1,Opv_Top_View_Target.apriltag_id2,
			                       Opv_Top_View_Target.width1,Opv_Top_View_Target.width2,
			                       Opv_Top_View_Target.height1,Opv_Top_View_Target.height2);	
		count_x--;
		flight_subtask_cnt[n]=5;
		flight_global_cnt[n]=0;			
		}
	}	
	//向前移动50CM
		else if(flight_subtask_cnt[n]==3)//检测起飞点悬停完毕后，飞向目标A——第21号作业区域
	{
		basic_auto_flight_support();//基本飞行支持软件
	  ncq_control_althold();
		SDK_DT_Send_Check(openmv_work_mode,UART3_SDK);
		if(flight_global_cnt[n]<Waypoint_Fix_Cnt1)//持续4*5ms=0.05s满足
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			
			if(dis_cm<=Waypoint_Fix_CM1)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
	else
  	{
			Horizontal_Navigation(0,58,0,RELATIVE_MODE,BODY_FRAME);	
			NCLink_Send_To_Firetruck(count_x,count_y,Opv_Top_View_Target.angle1,Opv_Top_View_Target.angle2,
			                      Opv_Top_View_Target.distance1,Opv_Top_View_Target.distance2,
			                      Opv_Top_View_Target.apriltag_id1,Opv_Top_View_Target.apriltag_id2,
			                       Opv_Top_View_Target.width1,Opv_Top_View_Target.width2,
			                       Opv_Top_View_Target.height1,Opv_Top_View_Target.height2);
      count_y--;			
		  flight_subtask_cnt[n]=5;
			flight_global_cnt[n]=0;
		}
		
	}
		
		//向后移动50CM
		else if(flight_subtask_cnt[n]==4)//
	{
		basic_auto_flight_support();//基本飞行支持软件
		ncq_control_althold();         //高度保持函数
		SDK_DT_Send_Check(openmv_work_mode,UART3_SDK);
		if(flight_global_cnt[n]<Waypoint_Fix_Cnt1)//持续4*5ms=0.05s满足
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM1)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
	else
  	{
			//向右移动50cm
			Horizontal_Navigation(0,-58,0,RELATIVE_MODE,BODY_FRAME);
		NCLink_Send_To_Firetruck(count_x,count_y,Opv_Top_View_Target.angle1,Opv_Top_View_Target.angle2,
			                      Opv_Top_View_Target.distance1,Opv_Top_View_Target.distance2,
			                      Opv_Top_View_Target.apriltag_id1,Opv_Top_View_Target.apriltag_id2,
			                       Opv_Top_View_Target.width1,Opv_Top_View_Target.width2,
			                       Opv_Top_View_Target.height1,Opv_Top_View_Target.height2);
    count_y++;			
		flight_subtask_cnt[n]=5;
		flight_global_cnt[n]=0;			
		}

	}	
	//对数组进行判断，确定应该如何移动
		else if(flight_subtask_cnt[n]==5)
	{	
		switch (steps[count-1])
		{
			case 1:
			flight_subtask_cnt[n]=1;
			break;
			case 2:
			flight_subtask_cnt[n]=2;
			break;
			case 3:
			flight_subtask_cnt[n]=3;
			break;
			default:
			flight_subtask_cnt[n]=4;
		}
		count++;
		flight_global_cnt[n]=0;
		if(count-1 == step1-13+xcount)
		{
			
			if(!(xcount==0))
			{
				flight_subtask_cnt[n]=6;
			}
			else 
			{
	     //if(stop[0][0]==6||stop[1][0]==6||stop[2][0]==6||stop[0][1]==0||stop[1][1]==0||stop[2][1]==0)
			if(stop[0][0]==0||stop[1][0]==0||stop[2][0]==0||stop[0][1]==8||stop[1][1]==8||stop[2][1]==8)
			{
				flight_subtask_cnt[n]=8;
			}
			else 
			 {
			  flight_subtask_cnt[n]=6;
			 }
		  }
	}
}
     else if(flight_subtask_cnt[n]==6)	
		 {
		 basic_auto_flight_support();//基本飞行支持软件
		
		if(flight_global_cnt[n]<Waypoint_Fix_Cnt1)//持续4*5ms=0.05s满足
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM1)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
	else
  	{
			Horizontal_Navigation(0,-58*(8-xcount),0,RELATIVE_MODE,BODY_FRAME);
		flight_subtask_cnt[n]=7;
		flight_global_cnt[n]=0;			
		}
   }
		 
	 
	else if(flight_subtask_cnt[n]==7)
	{
		basic_auto_flight_support();//基本飞行支持软件
		
			if(flight_global_cnt[n]<Waypoint_Fix_Cnt1)//持续4*5ms=0.05s满足
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM1)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
	else
  	{
			//向左移动50cm
		Horizontal_Navigation(-58*6,0,0,RELATIVE_MODE,BODY_FRAME);	
		flight_subtask_cnt[n]=10;
		flight_global_cnt[n]=0;			
		}
	}	
	
	
	
	
	
	
		else if(flight_subtask_cnt[n]==8)
	{
		basic_auto_flight_support();//基本飞行支持软件
		
			if(flight_global_cnt[n]<Waypoint_Fix_Cnt1)//持续4*5ms=0.05s满足
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM1)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
	else
  	{
			//向左移动50cm
		Horizontal_Navigation(-58*6,0,0,RELATIVE_MODE,BODY_FRAME);	
		flight_subtask_cnt[n]=9;
		flight_global_cnt[n]=0;			
		}
	}	
	     else if(flight_subtask_cnt[n]==9)	
		 {
		 basic_auto_flight_support();//基本飞行支持软件
		
		if(flight_global_cnt[n]<Waypoint_Fix_Cnt1)//持续4*5ms=0.05s满足
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM1)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
	else
  	{
			Horizontal_Navigation(0,-58*(8-xcount),0,RELATIVE_MODE,BODY_FRAME);
		flight_subtask_cnt[n]=10;
		flight_global_cnt[n]=0;			
		}
   }
		 
	else if(flight_subtask_cnt[n]==10)//原地降落
	{	
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
		OpticalFlow_Control_Pure(0);
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//高度控制
		//当巡航高度所在环境与地面环境差异偏大时，建议水平方向只使用速度控制降落，避免位置观测误差造成的降落点偏移
		//OpticalFlow_X_Vel_Control(0);
		//OpticalFlow_Y_Vel_Control(0);
	}
	
	/*俯视水平追踪
	else if(flight_subtask_cnt[n]==11)
	{
	   Top_APrilTag_Control_Pilot();//俯视OPENMV视觉水平追踪		
			Flight.yaw_ctrl_mode=ROTATE;
			Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];			
			Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//高度控制
	
	}
	
	*/
	else
	{
		basic_auto_flight_support();//基本飞行支持软件
	}
}
