#include "naobehavior.h"
#include <iostream>
#include <sys/time.h>

// void NaoBehavior::initRRT(VecPosition startPoint,int maxtIeration,double increasedStep)
// {

// }



void NaoBehavior::buildRRT(std::vector<VecPosition> paths,VecPosition &startPoint,VecPosition &goal,
								std::vector<VecPosition> &obstacles,double R,
						        int maxtIeration,double increasedStep)
{
	//需要优化的值

	double angleLeft,angleRight;
	angleLeft = angleRight = getLimitingAngleForward() * .9;
	//设置局部目标点
	VecPosition myPos = me,flagPosL(R,0,0);
	myPos.setZ(0);
    flagPosL = flagPosL.rotateAboutZ(- worldModel->getMyAngDeg() - angleLeft) + myPos;
    VecPosition tmp(goal);
    tmp.setZ(0);
	double angleGoal = myPos.getAngleBetweenPoints(flagPosL,tmp,true);
	// cout << tmp << "\n"; 
	VecPosition pGoal(R,0,0);
	// cout << angleGoal << " : L " << 180 + angleLeft << " R " << 2*angleLeft << "\n";
	if(angleGoal >  180 + angleLeft )
		pGoal = pGoal.rotateAboutZ(- worldModel->getMyAngDeg() - angleLeft) + myPos;
	else if(angleGoal > 2*angleLeft)
		pGoal = pGoal.rotateAboutZ(- worldModel->getMyAngDeg() + angleLeft) + myPos;
	else 
		pGoal = pGoal.rotateAboutZ(- worldModel->getMyAngDeg() + angleGoal - angleLeft) + myPos;

	tmp = VecPosition(R,0,0);
	tmp = tmp.rotateAboutZ(- worldModel->getMyAngDeg() - 180) + myPos;
	worldModel->getRVSender()->drawPoint("pgoal",pGoal,RVSender::MAGENTA);
	worldModel->getRVSender()->drawLine("behind",me,tmp,RVSender::MAGENTA);

	vector<vecPositionValue> RRTTreeForward;
	for(int i = 0;i < maxtIeration;i ++){
		//生成随机坐标点，由于问题的特殊性，引入最大可偏转Theta角
		double r = rand()%10000/10000.;
		
		// cout << r << "\n";
		break;
	}
	// cout << "test bigger (false)" << (p1 > p2） << "smaller (true)" << (p1<p2) << "\n";
}


//need to tune
//分离ball,opponent,teammate
//avoid teammate（opponent） : 自己(敌方)的球员不会影响p1与p2之间的直线距离
//PROXIMITY_THRESH ： 球员能影响的范围的半径

//obstacles ： 搜索范围内的所有障碍物
//num ：  搜索范围内的所有障碍物对应WorldObject中的枚举值
//p1,p2 : 待求两点之间距离
//为了适应多种情况
//method :
//method = 1 : 直线模式[障碍物间的阻碍按两点之间的直线计算]
//method = 2 : 矩形模式[障碍物按矩形[垂直两点的连线的两条直径]计算]
//method = 3 : 扇形模式[障碍物以p1为圆心，连线为半径的的扇形]

double NaoBehavior::agentsConnectDistance(std::vector<VecPosition> &obstacles,std::vector<int> &num,VecPosition &p1,VecPosition &p2,
												double PROXIMITY_THRESH,int method,bool avoidTeammate, bool avoidOpponent)
{
	SIM::Point2D p1tmp(p1),p2tmp(p2);
	if(p1.getDistanceTo(p2) < PROXIMITY_THRESH*1.5) return INFINITY;
	switch(method){
		case 1 :
			//直线模式
			for(unsigned int i = 0;i < obstacles.size();i ++){
				if( obstacles[i] == p1 || obstacles[i] == p2 ) continue;
				if( !avoidTeammate && num[i] >= WO_TEAMMATE1 && num[i] <= WO_TEAMMATE11 ) continue;
				if( !avoidOpponent && num[i] >= WO_OPPONENT1 && num[i] <= WO_OPPONENT11 ) continue;
				SIM::Circle c(obstacles[i],PROXIMITY_THRESH);
				if(c.haveIntersectionWithLine(p1tmp,p2tmp)) return INFINITY;
			}
			return p1tmp.getDistanceTo(p2tmp);
		case 2 :
			//矩形模式
			for(unsigned int i = 0;i < obstacles.size();i ++){
				if( obstacles[i] == p1 || obstacles[i] == p2 ) continue;
				if( !avoidTeammate && num[i] >= WO_TEAMMATE1 && num[i] <= WO_TEAMMATE11 ) continue;
				if( !avoidOpponent && num[i] >= WO_OPPONENT1 && num[i] <= WO_OPPONENT11 ) continue;
				VecPosition tmp = p2- p1;
				tmp.setMagnitude(1);//单位方向向量

				SIM::Point2D normal = SIM::Point2D(-tmp.getY(),tmp.getX());//单位法向量
				SIM::Circle c(obstacles[i],PROXIMITY_THRESH);
				SIM::Point2D t1 = p1tmp + normal*PROXIMITY_THRESH,t2 = p2tmp + normal*PROXIMITY_THRESH;
				worldModel->getRVSender()->drawLine("line21",t1.getX(),t1.getY(),t2.getX(),t2.getY());
				if(c.haveIntersectionWithLine(t1,t2)) return INFINITY;

				normal = SIM::Point2D(tmp.getY(),-tmp.getX());//单位法向量
				c = SIM::Circle(obstacles[i],PROXIMITY_THRESH);
				t1 = p1tmp + normal*PROXIMITY_THRESH;t2 = p2tmp + normal*PROXIMITY_THRESH;
				worldModel->getRVSender()->drawLine("line22",t1.getX(),t1.getY(),t2.getX(),t2.getY());
				if(c.haveIntersectionWithLine(t1,t2)) return INFINITY;
			}
			return p1tmp.getDistanceTo(p2tmp);
		case 3 :
			//扇形模式
			for(unsigned int i = 0;i < obstacles.size();i ++){
				if( obstacles[i] == p1 || obstacles[i] == p2 ) continue;
				if( !avoidTeammate && num[i] >= WO_TEAMMATE1 && num[i] <= WO_TEAMMATE11 ) continue;
				if( !avoidOpponent && num[i] >= WO_OPPONENT1 && num[i] <= WO_OPPONENT11 ) continue;
				VecPosition tmp = p2- p1;
				tmp.setMagnitude(1);//单位方向向量

				SIM::Point2D normal = SIM::Point2D(-tmp.getY(),tmp.getX());//单位法向量
				SIM::Circle c(obstacles[i],PROXIMITY_THRESH);
				SIM::Point2D t2 = p2tmp + normal*PROXIMITY_THRESH;
				worldModel->getRVSender()->drawLine("line31",p1.getX(),p1.getY(),t2.getX(),t2.getY());
				if(c.haveIntersectionWithLine(t2,p1tmp)) return INFINITY;

				normal = SIM::Point2D(tmp.getY(),-tmp.getX());//单位法向量
				c = SIM::Circle(obstacles[i],PROXIMITY_THRESH);
				t2 = p2tmp + normal*PROXIMITY_THRESH;
				worldModel->getRVSender()->drawLine("line32",p1.getX(),p1.getY(),t2.getX(),t2.getY());
				if(c.haveIntersectionWithLine(t2,p1tmp)) return INFINITY;
			}
			return p1tmp.getDistanceTo(p2tmp);
		default:
			return INFINITY;

	}
}

/*
lishang6257
设计目的（远程避障）：
1.当目标不再搜索的范围中的时候，即searchR < startPoint.getDistanceTo(goal)的时候，会返回一个较好的下一个移动目标点
2.在设计时还未考虑球在搜索范围中

为了解决这个问题，准备设计一个进程的避障searchR > startPoint.getDistanceTo(goal)
在UT的测试中,近距离的避障可以达到较好的效果，但是当通道过于狭窄时，容易造成振荡现象
思考如下：缩小PROXIMITY_THRESH的值
*/
VecPosition NaoBehavior::buildDijkstraForLongDistanceAvoid(std::vector<VecPosition> &paths,VecPosition &startPoint,VecPosition &goal,
                                std::vector<VecPosition> &obstacles,std::vector<int> &num,double searchR,
                                double PROXIMITY_THRESH,bool avoidTeammate, bool avoidOpponent)
{
	if(obstacles.size() == 0) return goal;

	int vexnum = obstacles.size() + 2;//将起点与终点加入邻接矩阵

	double **adjacency = new double*[vexnum]; //第一维，
	for(int i=0; i< vexnum; i++){
		adjacency[i] = new double[vexnum]; //分配第二维，每一行的空间。
		adjacency[i][i] = INFINITY;//初始化
	}
	bool *visit = new bool[vexnum];
	double *value = new double[vexnum];
	double *apath = new double[vexnum];//保存到达该节点的最短路径中做后一个节点

	//初始化障碍物的邻接矩阵
	for(unsigned int i = 0;i < obstacles.size();i ++){
		for(unsigned int j = 0;j < obstacles.size();j ++){
			if(i == j) continue;
			adjacency[j+1][i+1] = adjacency[i+1][j+1] = agentsConnectDistance(obstacles,num,obstacles[i],obstacles[j],PROXIMITY_THRESH,2/*矩形模式*/);
		}
	}
	//初始化起点邻接矩阵
	for(unsigned int i = 0;i < obstacles.size();i ++){
		adjacency[0][i+1] = adjacency[i+1][0] = agentsConnectDistance(obstacles,num,obstacles[i],startPoint,PROXIMITY_THRESH,2/*矩形模式*/);
	}
	//初始化终点的邻接矩阵
	adjacency[vexnum-1][0] = adjacency[0][vexnum - 1] = agentsConnectDistance(obstacles,num,goal,startPoint,PROXIMITY_THRESH,3/*扇形模式*/);//起点与终点
	for(unsigned int i = 0;i < obstacles.size();i ++){
		adjacency[vexnum - 1][i+1] = adjacency[i+1][vexnum-1] = agentsConnectDistance(obstacles,num,goal,obstacles[i],PROXIMITY_THRESH,3/*扇形模式*/);
	}


	// for(int i = 0;i < vexnum;i ++){
	// 	for(int j = 0;j < vexnum;j ++){
	// 		cout << adjacency[i][j] << "\t";
	// 	}
	// 	cout << "\n";
	// }
	// cout << "\n";

	//dijkstra
	int begin = 1;//起点所在位置，从[1..vexnum]
	//首先初始化我们的dis数组 
	for (int i = 0; i < vexnum; i++){
		//设置当前的路径
	 	value[i] = adjacency[begin - 1][i]; 
	 	visit[i] = false;
	 	if(value[i] != INFINITY) apath[i] = 0;
	 	else apath[i] = -1;
	} 
	paths.push_back(startPoint);//初始化
	//设置起点的到起点的路径为0 
	value[begin - 1] = 0; 
	visit[begin - 1] = true; 

	int count = 1; //计算剩余的顶点的最短路径（剩余this->vexnum-1个顶点） 
	while (count != vexnum) { 
	 	//temp用于保存当前dis数组中最小的那个下标 
	 	//min记录的当前的最小值 
	 	int temp=0; 
	 	double min = INFINITY; 
	 	for (int i = 0; i < vexnum; i++){ 
	 		if (!visit[i] && value[i]<min){ 
	 			min = value[i]; 
	 			temp = i; 
	 		} 
	 	} 
	 	//cout << temp + 1 << "  "<<min << endl; 
	 	//把temp对应的顶点加入到已经找到的最短路径的集合中 
	 	visit[temp] = true; 
	 	if(visit[vexnum - 1]) break;
	 	++count; 
	 	for (int i = 0; i < vexnum; i++){ 
	 		//注意这里的条件adjacency[temp][i]!=INFINITY必须加，不然会出现溢出，从而造成程序异常 
	 		if (!visit[i] && adjacency[temp][i]!=INFINITY && (value[temp] + adjacency[temp][i]) < value[i]){ 
	 			//如果新得到的边可以影响其他为访问的顶点，那就就更新它的最短路径和长度 
	 			value[i] = value[temp] + adjacency[temp][i]; 
	 			apath[i] = temp;
	 		} 
	 	} 
	} 

	// for(int i = 0;i < vexnum;i ++) cout <<  apath[i] << "\t";
	// cout << "\n";

	count = vexnum - 1;
	while(apath[count] != -1){
		if(count == vexnum - 1)
			paths.insert(paths.begin(),goal);
		else 
			paths.insert(paths.begin(),obstacles[count - 1]);
		
		count = apath[count];
	}
	paths.insert(paths.begin(),startPoint);

	if(paths.size() > 1){//有路
		VecPosition p2(paths[1]),p3;
		if(paths.size() > 2) p3 = VecPosition(paths[2]);
		else p3 = VecPosition(goal);
		//求角平分向量
		p3 = goal - p2;
		p2 = startPoint - p2;
		p2.setZ(0);p3.setZ(0);
		p3.setMagnitude(1);p2.setMagnitude(1);
		VecPosition res = p2+p3;
		if( res != VecPosition(0,0,0) ){
			res.setMagnitude(1);
			res = paths[1]+res*PROXIMITY_THRESH;//假定最小为一
			// res = collisionAvoidance(avoidTeammate /*teammate*/, avoidOpponent/*opponent*/, false/*ball*/, PROXIMITY_THRESH/*proximity thresh*/, PROXIMITY_THRESH/*collision thresh*/, res, true/*keepDistance*/);
			return res;
		}
		// else if(rand()%2 > 0){
		// 	cout << "bad\n";
		// 	res =  paths[1]+VecPosition(-p2.getY(),p2.getX(),0)*PROXIMITY_THRESH; //p2,p3共线，随机取一个法向量
		// 	res = collisionAvoidance(avoidTeammate /*teammate*/, avoidOpponent/*opponent*/, false/*ball*/, .8/*proximity thresh*/, .5/*collision thresh*/, res, true/*keepDistance*/);
		// 	return res;
		// }
		else{
			cout << "bad\n";
			res =  paths[1]+VecPosition(p2.getY(),-p2.getX(),0)*PROXIMITY_THRESH;
			// res = collisionAvoidance(avoidTeammate /*teammate*/, avoidOpponent/*opponent*/, false/*ball*/, PROXIMITY_THRESH/*proximity thresh*/, PROXIMITY_THRESH/*collision thresh*/, res, true/*keepDistance*/);
			return res;
		}


	}
	cout << "error\n";
	return goal;

}

