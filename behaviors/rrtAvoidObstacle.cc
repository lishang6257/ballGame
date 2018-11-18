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

double NaoBehavior::agentsConnectDistance(std::vector<VecPosition> &obstacles,std::vector<int> &num,VecPosition &p1,VecPosition &p2,
												double PROXIMITY_THRESH,bool avoidTeammate, bool avoidOpponent)
{
	SIM::Point2D p1tmp(p1),p2tmp(p2);
	for(unsigned int i = 0;i < obstacles.size();i ++){
		if(obstacles[i] == p1 || obstacles[i] == p2) continue;
		SIM::Circle c(obstacles[i],PROXIMITY_THRESH);
		if(c.haveIntersectionWithLine(p1tmp,p2tmp)) return INFINITY;
	}
	return p1tmp.getDistanceTo(p2tmp);
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
void NaoBehavior::buildDijkstra(std::vector<VecPosition> &paths,VecPosition &startPoint,VecPosition &goal,
                                std::vector<VecPosition> &obstacles,std::vector<int> &num,double searchR,
                                double PROXIMITY_THRESH,bool avoidTeammate, bool avoidOpponent)
{
	if(obstacles.size() == 0) return ;

	obstacles.push_back(goal);//将目标添加到邻接矩阵

	int vexnum = obstacles.size() + 1;

	double **adjacency = new double*[vexnum]; //第一维，
	for(int i=0; i< vexnum; i++){
		adjacency[i] = new double[vexnum]; //分配第二维，每一行的空间。
		adjacency[i][i] = INFINITY;//初始化
	}
	bool *visit = new bool[vexnum];
	double *value = new double[vexnum];
	double *apath = new double[vexnum];//保存到达该节点的最短路径中做后一个节点

	for(unsigned int i = 0;i < obstacles.size();i ++){
		for(unsigned int j = 0;j < obstacles.size();j ++){
			if(i == j) continue;
			adjacency[j+1][i+1] = adjacency[i+1][j+1] = agentsConnectDistance(obstacles,num,obstacles[i],obstacles[j],PROXIMITY_THRESH);
		}
	}
	for(unsigned int i = 0;i < obstacles.size();i ++){
		adjacency[0][i+1] = adjacency[i+1][0] = agentsConnectDistance(obstacles,num,obstacles[i],startPoint,PROXIMITY_THRESH);
	}

	for(int i = 0;i < vexnum;i ++){
		for(int j = 0;j < vexnum;j ++){
			cout << adjacency[i][j] << "\t";
		}
		cout << "\n";
	}
	cout << "\n";

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
	for(int i = 0;i < vexnum;i ++) cout <<  apath[i] << "\t";
	cout << "\n";

	obstacles.erase(obstacles.end());//复原obstacles;

}
