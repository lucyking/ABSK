#ifndef __ROUTE_H__
#define __ROUTE_H__

#define MAXVEX 600
#define MAXNUM 65535
#define ERROR 0
#define OK 1
typedef struct elem
{
    int Edge;
    int weigt;
}elem;

int Xhuisu(int S,int T);
int XGetResult(unsigned short *result);
int XCheckRe(int i);
int XCheckPass();

int WeigtSum();
bool CheckT();
int Dijkstra(int Vexnum,int Vbegin, int *P,int *D);
int GetResult(unsigned short *result);
bool CheckPass(int S,int T,int *Path);
bool FindChild(int S);
void ClearRu(int S,int y);
void Push(int S,int T);
void Pop();//弹出
bool FindWay(int S,int T);

void TopoToTable(int topoint[5000][4],int edgen,int *max);

void search_route(char *graph[5000], int edge_num, char *condition);

#endif
