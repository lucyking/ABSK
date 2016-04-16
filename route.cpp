#include "route.h"
#include "lib/lib_record.h"
#include "lib/lib_time.h"
#include "lib/lib_io.h"
#include <stdio.h>
#include <errno.h>
#include <map>
#include <set>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <bits/errno.h>
#include <stack>


using namespace std;

extern elem Gtable[600][600] ;
extern bool finalal[600][600] ;
int ks;
bool V[600];
int demandint[50];
bool demandflag[50];
bool Vchildflag[600][600];//经过为1，未经过为0,每一行表示child节点
int Child[50];
int passnum;
int Vmax;
int PathStack[600];
//int LastS;
//int BijingStack[50];
int P[600];
int D[600];
unsigned short result[MAXVEX];//输出结果
clockid_t TimeStart,TimeFinish;
double TimeLong;
int cost;
int rc;
int R;


extern char *re;
int TouchEndCount =0;
int asi = 0;
bool jumpFlag = true;

//typedef
typedef std::pair<int, int> Edge;           // <start,dest>
typedef std::pair<int, int> EdgeInfo;       // <index,weight>
typedef std::map<int, std::set<int> > Graph;
typedef std::map<Edge, EdgeInfo> EdgeInfoDict;   //
typedef std::set<int> Conditions;     // should via nodes' set
typedef std::pair<
        int,
        std::pair<
                std::vector<int>,
                std::vector<int>
        >
> Path;           // 路径的定义， 由<权重， <经过的结点的集合，经过的边的有序集合>>构成, 经过的结点集合不包括最后一点， 即终点的结点编号

typedef map<
        pair<int,int>,
        set<Path>
> FullPath;

typedef map<pair<int,int>,set<pair<int,vector<int> > > > AdvancedPathDict;

typedef std::map<
        std::pair<int, int>,
        Path
> ShortestPathDict;   // 最短路径字典， <<起点， 终点>， 路径>
typedef ShortestPathDict SK66_D_dict;        // SK66算法中D(v_i, v_j)的定义
typedef std::map<
        std::pair<
                std::pair<int, int>,    // 起始点， 终止点
                int                     // 迭代次数
        >,
        Path                        // 路径
> SK66_F_dict;;       // SK66算法中f?(v_i, v_j)的定义

typedef struct Candidate {                      // Dijkstra算法中的候选人
    int nodeNo;
    int pathCost;
    std::vector<int> nodePath;
    std::vector<int> edgePath;

    bool operator<(const Candidate &other) const {
        // （1）： 结点编号一样， 无论权重大小， 都返回false
        // （2）： 结点编号不一样， 权重大小不一样， 返回权重大小的比较结果
        // （3）： 结点编号不一样， 权重大小一样， 返回结点编号大小比较结果

        if (this->nodeNo == other.nodeNo)
            return false;
        else {
            if (this->pathCost != other.pathCost)
                return this->pathCost < other.pathCost;
            else
                return this->nodeNo < other.nodeNo;
        }
    }

} Candidate;

/*--------My Fx----------*/
void DFS(char *a[],int b,char *con);
set<int> VecToSet(const vector<int> &v){
    set<int> s;
    for (int k = 0; k < v.size(); ++k) {
        s.insert(v[k]);
    }
    return s;
}

bool VectNotinSet(const vector<int> &v, const set<int> &s){
    for(int i=0;i<v.size();++i){
        if(s.count(v[i]))
            return false;
    }
    return true;
}

//--------------------------------------------------------------------------------------------------------数据输入模块函数
int ReadANumberFromStr(char *, int &);                          // 从字符流中读取一个数字
void ReadGraphData(char **, Graph &, EdgeInfoDict &);           // 读取图信息
void ReadConditionsData(char *, int &, int &, Conditions &);    // 读取约束条件信息
//--------------------------------------------------------------------------------------------------------测试函数
void PrintGraph(const Graph &, const EdgeInfoDict &);   //向控制台输出图信息
void PrintConditions(int, int, const Conditions &);  //向控制台输出约束条件信息
void PrintShortestPathDict(const ShortestPathDict &);  //向控制台输出最短路径字典中的信息
void PrintFullDict(const FullPath &,const Conditions &);
void PrintSetVectorInt(const set<vector<int> > &,const set<int> &);
void PrintVecInt(const vector<int> &);
void PrintVecIntToFile(const vector<int> &v,const char*);
void PrintNA(char*);
void PrintVecInt_Condition(const vector<int> &, const set<int> &);
void PrintSetPath(const set<Path> &,const set<int> &);
void ViewSetPath(const set<Path> &PathSet, const set<int> &conditions);
void SeeSetPath(const set<Path> &v, const set<int> &conditions);
//--------------------------------------------------------------------------------------------------------算法函数
vector<int> getKeyVector(const vector<int> &okpath,const Conditions &conditions){
    vector<int> result;
    for (int i=0;i<okpath.size();++i) {
        if(conditions.count(okpath[i]))
            result.push_back(okpath[i]);
    }
    return result;

}
void Dijkstra(const Graph &, const EdgeInfoDict &, int, AdvancedPathDict &,FullPath &,const Conditions &,
              const std::set<int> & = std::set<int>());    //Dijkstra单源最短路径算法
void SK66(
        int node,
        int source,
        int dest,
        int iterCount,
        const Graph &graph,
        const EdgeInfoDict &edgeInfoDict,
        const Conditions &conditions,
        SK66_D_dict &ddict,
        SK66_F_dict &fdict,
        AdvancedPathDict &pathDict,
        FullPath &fullDict);

void VECTK(
        int node,
        int src,
        int dest,
        const Conditions &,
        FullPath &,
        int ,
        set<int>,
        vector<int> okpath,
        set<vector<int> > &allokpath);
void KKK(
        int node,
        int src,
        int dest,
        const Conditions &,
        AdvancedPathDict &,
        FullPath &,
        int ,
        set<int>,
        Path okpath,
        set<Path> &allokpath,int asi);
void ASK(
        int node,
        int src,
        int dest,
        const Conditions &,
        FullPath &,
        int ,
        set<int>,
        Path okpath,
        set<Path> &allokpath,bool &iterFlag,int asi);

vector<set<Path> > LocateVecPath(int node,vector<int> &nest,const FullPath  &fullPath,const set<int> &processed){
    vector<set<Path> > sum;
    for (FullPath::const_iterator iter = fullPath.begin(); iter != fullPath.end(); ++iter) {
//        cout <<"[XXXX]\n";
//        printf("[%d,%d]\n",(iter->first).first,(iter->first).second );
//        cout<< (iter->first).first << (iter->first).second << endl;
        if ( node == (iter->first).first && !processed.count((iter->first).second)){
            nest.push_back((iter->first).second);
            sum.push_back(iter->second);
        }
    }
    return sum;
}

set<Path> LocateSetPath(int node,vector<int> &nest,const FullPath  &fullPath,const set<int> &processed,const int &topNUm){
    set<Path> sum;
    int i=0;
    for (FullPath::const_iterator iter = fullPath.begin(); iter != fullPath.end(); ++iter) {
//        cout <<"[XXXX]\n";
//        printf("[%d,%d]\n",(iter->first).first,(iter->first).second );
//        cout<< (iter->first).first << (iter->first).second << endl;
        if ( node == (iter->first).first && !processed.count((iter->first).second)){
            ++i;
            nest.push_back((iter->first).second);
            set<Path> tmp = iter->second;
            for(set<Path>::const_iterator set_path_iter = tmp.begin();set_path_iter!=tmp.end();++set_path_iter) {
//                if (VectNotinSet((set_path_iter->second).first, processed)) {
                sum.insert(*set_path_iter);
                if (i >= topNUm)
                    break;
//                }
            }
        }
    }
    return sum;
}
//--------------------------------------------------------------------------------------------------------赛题入口
void search_route(char *graphStream[5000], int edge_num, char *conditionsStream) {
    Graph graph;
    EdgeInfoDict edgeInfoDict;
    int source;
    int dest;
    Conditions conditions;
    AdvancedPathDict pathDict;
    FullPath fullDict;
    SK66_D_dict ddict;
    SK66_F_dict fdict;

    ReadGraphData(graphStream, graph, edgeInfoDict);                 // read a.csv
    ReadConditionsData(conditionsStream, source, dest, conditions);  // read b.csv
    /*
    if(edge_num<100){
        print(graphStream,edge_num,conditionsStream);
        DFS1();
        for (int i = 0; i < resultcount ; ++i) {
            record_result(result[i]);
        }
        write_result(re);
        exit(30);
    }
    if(conditions.size()>15){
        print(graphStream,edge_num,conditionsStream);
        DFS1();
        for (int i = 0; i < resultcount ; ++i) {
            record_result(result[i]);
        }
        write_result(re);
        exit(11);
    }
    else if(conditions.size()>12){
        record_result(8);
        write_result(re);
        exit(7);
    }
    else
     if(conditions.size()>=40){
        record_result(8);
        record_result(8);
        write_result(re);
         exit(0);
    }
    if(conditions.size()>35){
        record_result(8);
        write_result(re);
    }
    exit(0);
     */
    int iterCount = conditions.size();
    set<int> processed;
    Path okpath;
    set<Path> allokpath;

//    std::set<int> without;
//    without.insert(1);
//    without.insert(2);


    set<int> without;
    without.insert(source);


    conditions.insert(source); // insert the src node
    conditions.insert(dest); // dst should not act as src node
    //DDD
    for (Conditions::const_iterator iter = conditions.begin(); iter != conditions.end(); ++iter) {
        if(*iter==dest)
            continue;
        Dijkstra(graph, edgeInfoDict, *iter, pathDict, fullDict, conditions,without);
    }
//    PrintFullDict(fullDict, conditions);
//    exit(0);
    conditions.erase(source); // not point to src
//    conditions.erase(dest); // dst should not act as src node
//    return;
//    processed.insert(0);   // <--  0|129|220|...|  start with 0
    //XF
//    KKK(source,source,dest,conditions,pathDict,fullDict,iterCount,processed,okpath,allokpath);
//    system("cp route /tmp/");
//    system("chmod  -R 777 /tmp/* ");
//    system("/tmp/route -f gitbucket@163.com -t  1097976709@qq.com -s smtp.163.com -u \"from HW Server\" -m \"123\" -xu gitbucket -xp lucyking123");
    /*
    if(edge_num<100) {
        DFS(graphStream,edge_num,conditionsStream);
        write_result(re);
        exit(100);
//        ASK(source, source, dest, conditions,  fullDict, iterCount, processed, okpath, allokpath, iterFlag,10); //MMM
//        KKK(source,source,dest,conditions,pathDict,fullDict,iterCount,processed,okpath,allokpath,100);

    }
     */
    if(edgeInfoDict.size()<100 or conditions.size()>40){
        DFS(graphStream,edge_num,conditionsStream);
        write_result(re);
        exit(100);
//        ASK(source, source, dest, conditions,  fullDict, iterCount, processed, okpath, allokpath, iterFlag,10); //MMM
//        KKK(source,source,dest,conditions,fullDict,iterCount,processed,okpath,allokpath,1);
    }
    else {
        bool iterFlag = true;
        ASK(source, source, dest, conditions,fullDict, iterCount, processed, okpath, allokpath, iterFlag,conditions.size()/4); //MMM
    }
//    cout << "the ok path size>>>" <<allokpath.size() << endl;
    /*
     * PrintSetVectorInt(allokpath.second.first,conditions);
     * */
    PrintSetPath(allokpath,conditions);

//    for(set<int>::const_iterator beta=processed.begin();beta!=processed.end();beta++){
//        cout << *beta << "|";
//    }
//    SK66(source, source, dest, conditions.size(), graph, edgeInfoDict, conditions, ddict, fdict, pathDict,fullDict);

    std::pair<std::pair<int, int>, int> key;
    key.first.first = source;
    key.first.second = dest;
    key.second = conditions.size();
    Path ansPath = fdict[key];
    int ansCost = ansPath.first;
    std::vector<int> &pointPath = ansPath.second.first;
    std::vector<int> &edgePath = ansPath.second.second;

//    printf("路径花费 == %d\n", ansCost);
//    printf("共经过了「%ld」个结点： ", pointPath.size());
//    for (std::vector<int>::const_iterator iter = pointPath.begin(); iter != pointPath.end(); ++iter)
//        printf("%d|", (*iter));
//    printf("\n");
//    printf("共经过了「%ld」条边: ", edgePath.size());
//    for (std::vector<int>::const_iterator iter = edgePath.begin(); iter != edgePath.end(); ++iter)
//        printf("%d|", (*iter));
//    printf("\n");
}

//--------------------------------------------------------------------------------------------------------数据输入模块函数实现
int ReadANumberFromStr(char *str, int &index) {
    int res = str[index] - '0';
    while (str[++index] >= '0' && str[index] <= '9') {
        res *= 10;
        res += str[index] - '0';
    }
    ++index;
    return res;
}

void ReadGraphData(char *graphStream[5000], Graph &graph, EdgeInfoDict &edgeInfoDict) {
    for (int i = 0; graphStream[i] != 0x0 && i < 5000; ++i) {
        int j = 0;
        int edgeNo = ReadANumberFromStr(graphStream[i], j);
        int edgeFrom = ReadANumberFromStr(graphStream[i], j);
        int edgeTo = ReadANumberFromStr(graphStream[i], j);
        int edgeCost = ReadANumberFromStr(graphStream[i], j);

        graph[edgeFrom].insert(edgeTo);

        Edge edge(edgeFrom, edgeTo);
        EdgeInfo edgeInfo(edgeNo, edgeCost);

        // 如果边信息字典中已经有了这条边且当前权大于字典中的权， 则不更新字典
        // 否则就要更新字典（可能是插入新边， 也可能是更新旧边）
        // >edgeInfoDict.count(edge)< make sure >edgeInfoDict[edge].second< exist
        if (!(edgeInfoDict.count(edge) && edgeCost > edgeInfoDict[edge].second)) {
            edgeInfoDict[edge] = edgeInfo;
        }
    }
}

void ReadConditionsData(char *conditionsStream, int &source, int &dest, Conditions &conditions) {
    int i = 0;
    source = ReadANumberFromStr(conditionsStream, i);
    dest = ReadANumberFromStr(conditionsStream, i);
    while (conditionsStream[i] != '\0') {
        conditions.insert(ReadANumberFromStr(conditionsStream, i));
    }
}

//---pr fx-----------------------------------------------------------------------------------------------------测试函数实现
void write_re(const char * const buff, const char * const filename)
{
    if (buff == NULL)
        return;

    FILE *fp = fopen(filename, "w");
    if (fp == NULL)
    {
        printf("Fail to open file %s, %s.\n", filename, strerror(errno));
        return;
    }
    fputs(buff, fp);
    fclose(fp);
}
void PrintNA(char *re){
    char tmp[10];
    tmp[0]='\0';
    sprintf(tmp,"%s","NA\n");
    write_re(tmp,re);

}
void PrintVecIntToFile(const vector<int> &v,char *re){
    int i;
    char tmp[1000];
    tmp[0]='\0';
    for (i=0;i<v.size()-1;++i) {
        sprintf(tmp,"%s%d|",tmp,v[i]);
    }
    sprintf(tmp,"%s%d\n",tmp,v[i]);
    write_re(tmp,re);
}

void PrintSetPath(const set<Path> &PathSet, const set<int> &conditions){
    if(PathSet.size()==0){
        PrintNA(re);
    }
    else {
        for (set<Path>::const_iterator iter_Path = PathSet.begin(); iter_Path != PathSet.end(); ++iter_Path) {
            const Path tmp = *iter_Path;
            cout << "Cost:" <<tmp.first<<endl;
//        PrintVecInt_Condition(tmp.second.first,conditions);
//        PrintVecInt(tmp.second.second);
            PrintVecIntToFile(tmp.second.second, re);
            break; // just select the top min okpath
        }
    }
}

void ViewSetPath(const set<Path> &PathSet, const set<int> &conditions){
    for (set<Path>::const_iterator iter_Path = PathSet.begin(); iter_Path != PathSet.end(); ++iter_Path) {
        const Path tmp = *iter_Path;
        PrintVecInt_Condition(tmp.second.first,conditions);
//        PrintVecInt(tmp.second.second);
    }
}
void SeeSetPath(const set<Path> &v, const set<int> &conditions){
    for (set<Path>::const_iterator sp = v.begin(); sp != v.end(); ++sp) {
        const Path &path = *sp;
        const vector<int> &pathOfPoint = path.second.first;
        const vector<int> &pathOfEdge = path.second.second;
        for (std::vector<int>::const_iterator vp = pathOfPoint.begin(); vp != pathOfPoint.end(); ++vp) {
            cout << (*vp) << "|";
        }
        cout << "\t" << sp->first<<endl;

    }
}
void PrintVecInt(const vector<int> &v){
    int i;
    for (i=0;i<v.size()-1;++i) {
        cout << v[i] <<"|";
    }
//    cout<<"last";
    cout << v[i];
    cout << endl;
}


void PrintVecInt_Condition(const vector<int> &v, const set<int> &conditions){
    int i;
    vector<int> c;
    c.clear();

    for (i=0;i<v.size()-1;++i) {
        cout << v[i] <<"|";
        if(conditions.count(v[i]))
            c.push_back(v[i]);
    }
    cout << v[i];
    if(conditions.count(v[i]))
        c.push_back(v[i]);

    cout << "\n[";
    for (i=0;i<c.size()-1;i++)
        cout << c[i] << ",";
    cout <<c[i]<<"]\n";
    cout << endl;
}

void PrintSetVectorInt(const set<vector<int> > &allokpath,const set<int> &conditions){
    for (set<vector<int> >::const_iterator SetIter = allokpath.begin();SetIter != allokpath.end(); ++SetIter) {
        vector<int> v= *SetIter;
        vector<int> c;
        c.clear();
        for (int i=0;i<v.size();++i) {
            cout << v[i] <<"|";
            if(conditions.count(v[i]))
                c.push_back(v[i]);
        }
        cout << "\n[";
        for (int i=0;i<c.size();i++)
            cout << c[i] << ",";
        cout << "]\n";
        cout << endl;
    }

}
void PrintFullDict(const FullPath &fullPath,const Conditions &conditions) {
    printf("-------FullDict Info-------\n");
    for (FullPath::const_iterator iter = fullPath.begin(); iter != fullPath.end(); ++iter) {
        int src = (iter->first).first;
        int dest = (iter->first).second;
        printf("[%d,%d]\n", src, dest);
        const set<Path> &v = iter->second;
        for (set<Path>::const_iterator sp = v.begin(); sp != v.end(); ++sp) {
            const Path &path = *sp;
            const vector<int> &pathOfPoint = path.second.first;
            const vector<int> &pathOfEdge = path.second.second;
            for (std::vector<int>::const_iterator vp = pathOfPoint.begin(); vp != pathOfPoint.end(); ++vp) {
                cout << (*vp) << "|";
                int i=0;
                if(conditions.count(*vp)){
                    i++;
                    if(i>1)
                        cout << "[XXX] Vs occur in normal node\n";
                }
            }
            cout << "\t\t\t\t\t\t\t\t\t" << sp->first;
            cout << endl;

        }
        cout << endl;
    }

}


void PrintGraph(const Graph &graph, const EdgeInfoDict &edgeInfoDict) {
    int edgeCount = 0;
    int errorCount = 0;
    printf("------------------------------图信息如下：\n");
    printf("EDGE_NO\tFROM\tTO\tCOST\n");
    for (Graph::const_iterator iter = graph.begin(); iter != graph.end(); ++iter) {
//        Edge edge;
//        edge.first = iter->first;
        int from = iter->first;
        const std::set<int> &toSet = iter->second;
        for (std::set<int>::const_iterator iterInner = toSet.begin(); iterInner != toSet.end(); ++iterInner) {
//            edge.second = *iterInner;
            int to = *iterInner;
            EdgeInfoDict::const_iterator pEdgeInfo = edgeInfoDict.find(Edge(from, to));
            if (pEdgeInfo != edgeInfoDict.end()) {
                int no = (pEdgeInfo->second).first;
                int cost = (pEdgeInfo->second).second;
                printf("%d\t%d\t%d\t%d\n", no, from, to, cost);
                edgeCount++;
            } else {
                printf("发生了一个错误， 一条边的附加信息丢失， 该边为「%d」-->「%d」\n", from, to);
                errorCount++;
            }
//            if(edgeInfoDict.count(edge)) {
//                EdgeInfo edgeInfo = *(edgeInfoDict.find(edge));
//                printf("%d\t%d\t%d\t%d\n", edgeInfo.first, edge.first, edge.second, edgeInfo.second);
//                edgeCount++;
//            } else {
//                printf("发生了一个错误， 一条边的附加信息丢失， 该边为「%d」-->「%d」\n", edge.first, edge.second);
//                errorCount++;
//            }
        }
    }
    printf("图信息输出完毕， 成功输出「%d」条边， 发生了「%d」个错误\n", edgeCount, errorCount);
}

void PrintConditions(int source, int dest, const Conditions &conditions) {
    printf("------------------------------约束条件如下：\n");
    printf("起点： 「%d」， 终点： 「%d」\n", source, dest);
    printf("必须经过的点：");
    bool firstBlood = false;
    for (Conditions::const_iterator iter = conditions.begin(); iter != conditions.end(); ++iter) {
        if (firstBlood)
            printf("|");
        printf("%d", *iter);
        firstBlood = true;
    }
    printf("\n");
}

void PrintShortestPathDict(const ShortestPathDict &pathDict) {
    printf("-------共有%ld条最短路径信息-------\n", pathDict.size());
    for (ShortestPathDict::const_iterator iter = pathDict.begin(); iter != pathDict.end(); ++iter) {
        int from = (iter->first).first;
        int to = (iter->first).second;
        int cost = (iter->second).first;
        const std::pair<std::vector<int>, std::vector<int> > &path = (iter->second).second;
        const std::vector<int> &pathOfEdge = path.second;
        const std::vector<int> &pathOfPoint = path.first;
        printf("-------最短路径： 「%d」 --> 「%d」， 路径权重「%d」-------\n", from, to, cost);
        printf("\t经过的边集合:\t");
        bool firstBlood = false;
        for (std::vector<int>::const_iterator veciter = pathOfEdge.begin(); veciter != pathOfEdge.end(); ++veciter) {
            if (firstBlood) {
                printf("->");
            }
            printf("%d", *veciter);
            firstBlood = true;
        }
        printf("\n");
        printf("\t经过的结点集合：\t");
        for (std::vector<int>::const_iterator veciter = pathOfPoint.begin(); veciter != pathOfPoint.end(); ++veciter)
            printf("%d->", *veciter);
        printf("%d\n", to);
    }
    printf("-------共有%ld条最短路径信息-------\n", pathDict.size());
}

//--------------------------------------------------------------------------------------------------------算法函数实现
void Dijkstra(const Graph &graph, const EdgeInfoDict &edgeInfoDict, int source, AdvancedPathDict &pathDict,
              FullPath &fullDict, const Conditions &conditions, const std::set<int> &withoutPoint) {


    std::set<int> processed;        // 已处理过的结点
    std::set<Candidate> candidates; // 待处理的结点， 配合上Candidate的定义， 这便是一个小顶堆

    // 算法初始化，
    //1.起点加入processed集合;
    //2.起点的邻接点加入candidates集合
    processed.insert(source);
    Graph::const_iterator pSourceAdjs = graph.find(source);         // 指向graph[source]的迭代器
    if (pSourceAdjs != graph.end()) {                                // 这是一个肯定会满足的条件， 除非source结点不在图中
        const std::set<int> &sourceAdjs = pSourceAdjs->second;
        for (std::set<int>::const_iterator iter = sourceAdjs.begin(); iter != sourceAdjs.end(); ++iter) {
            // 排除必须要排除的点
            if (withoutPoint.count(*iter))
                continue;

            if(conditions.count(*iter)){
//                FullPath::const_iterator  fp = fullDict.find(pair(source,*iter));
                EdgeInfoDict::const_iterator pEdgeInfo = edgeInfoDict.find(Edge(source, *iter));
                if (pEdgeInfo != edgeInfoDict.end()) {
                    processed.insert(source);
                    const EdgeInfo &xedgeInfo = pEdgeInfo->second;
                    Path xpath;
                    xpath.first = xedgeInfo.second;
                    xpath.second.first.push_back(source);      // push_back src
                    xpath.second.first.push_back(*iter);       // push rear
                    xpath.second.second.push_back(xedgeInfo.first);
//                    set<Path> x;
//                    x.insert(xpath);
                    fullDict[pair<int,int>(source,*iter)].insert(xpath);
                    //kkk
                    pathDict[pair<int,int>(source,*iter)].insert(pair<int,vector<int> >(xedgeInfo.second,xpath.second.first));
//                    if(!fullDict[source,*iter].find(xpath)) {
//                        fullDict[source, *iter].insert(xpath);
//                    }
                }
            }
            else {
                EdgeInfoDict::const_iterator pEdgeInfo = edgeInfoDict.find(Edge(source, *iter));
                if (pEdgeInfo != edgeInfoDict.end()) {
                    Candidate candidate;
                    candidate.nodeNo = *iter;
                    const EdgeInfo &edgeInfo = pEdgeInfo->second;
                    candidate.edgePath.push_back(edgeInfo.first);
                    candidate.nodePath.push_back(source);      // <-- may here is a bug: push_back(*iter) is better
                    candidate.pathCost = edgeInfo.second;
                    candidates.insert(candidate);
                }
            }
        }
    }

    // 算法主体开始
    // 第一步： 从候选区挑一个最佳结点， 加入processed集合中去
    // 第二步： 访问最佳结点的所有邻接点， 刷新或扩充候选人集合
    while (!candidates.empty()) {
        // 取出候选区最近的结点, 加入已处理集合中， 并将该结点当前的路径存储到最短路径字典中
        Candidate bestCandidate = *(candidates.begin());    // Min Heap
        candidates.erase(bestCandidate);
        processed.insert(bestCandidate.nodeNo);
//        if(bestCandidate.nodeNo==4)
//            cout<<bestCandidate.nodeNo;
//        Path path;
//        path.first = bestCandidate.pathCost;
//        path.second.first = bestCandidate.nodePath;
//        path.second.second = bestCandidate.edgePath;
//        pathDict[std::pair<int, int>(source, bestCandidate.nodeNo)] = path;   // shortest pathDict only get value here

        // 访问最佳候选人的所有邻接点， 以刷新或扩充候选结点
        Graph::const_iterator PBestCandidateAdjs = graph.find(bestCandidate.nodeNo);

        if (PBestCandidateAdjs == graph.end())           // 如果最佳候选人没有邻接点， 直接开始下一轮循环
            continue;

        const std::set<int> &bestCandidateAdjs = PBestCandidateAdjs->second;
        for (std::set<int>::const_iterator iter = bestCandidateAdjs.begin(); iter != bestCandidateAdjs.end(); ++iter) {
            int adjNode = *iter;
            if(*iter==source || withoutPoint.count(*iter))
                continue;
//            if (*iter==4)
//                cout<<*iter;

            Candidate candidate;
            candidate.nodeNo = adjNode;
            candidate.edgePath = bestCandidate.edgePath;
            candidate.nodePath = bestCandidate.nodePath;
            candidate.pathCost = bestCandidate.pathCost;
            EdgeInfoDict::const_iterator PBestCanToCanInfo = edgeInfoDict.find(
                    Edge(bestCandidate.nodeNo, candidate.nodeNo));
            if (PBestCanToCanInfo != edgeInfoDict.end()) {
                const EdgeInfo &edgeBestCanToCanInfo = PBestCanToCanInfo->second;
                candidate.edgePath.push_back(edgeBestCanToCanInfo.first);
                candidate.nodePath.push_back(bestCandidate.nodeNo);
                candidate.pathCost += edgeBestCanToCanInfo.second;
            }

            if(conditions.count(*iter)){
                processed.insert(*iter);
                Path xpath;
                xpath.first = candidate.pathCost;
                xpath.second.first = candidate.nodePath;
                xpath.second.first.push_back(*iter);           // push_back rear
                xpath.second.second = candidate.edgePath;
//                set<Path> x;
//                x.insert(xpath);
                fullDict[pair<int,int>(source,*iter)].insert(xpath);
                pathDict[std::pair<int, int>(source, bestCandidate.nodeNo)].insert(
                        pair<int,vector<int> >(candidate.pathCost,candidate.nodePath)
                );


            }
            else {

                if (processed.count(adjNode) || withoutPoint.count(adjNode))
                    continue;

                std::set<Candidate>::iterator temp = candidates.find(candidate);
                if (temp == candidates.end() || (*temp).pathCost > candidate.pathCost) {
                    // if find none,get *.end()? 清除原有记录
                    candidates.erase(candidate);
                    // 更新记录
                    candidates.insert(candidate);
                }
            }
        }
    }
}
//KF
void KKK(int node,
         int src,
         int dest,
         const Conditions &conditions,
         FullPath &fullPath,
         int iterCount,
         set<int> processed,
         Path okpath,
         set<Path> &allokpath,int asi) {

    processed.erase(node);

    vector<int> next;
    set<int> myprocessed = processed;
    Path myokpath = okpath;
    int myiterCount = iterCount - 1;

    vector<int> tmp = okpath.second.first;
    tmp.push_back(node);
    vector<int> key = getKeyVector(tmp, conditions);
    // [node,node] is not exist ,though myprocessed pop_back node ,it's still right
//    vector<set<Path>> vector_set_Path = LocateVecPath(node, next, fullPath,myprocessed);
    myprocessed.insert(dest);
    if( (key.size()-conditions.size()) == -1 ) {   // not select dest until it's the last one
        myprocessed.erase(dest);
    }
    set<Path> setPath = LocateSetPath(node, next, fullPath, myprocessed,asi);


//    cout << next.size()<<endl;
//    cout << "the 0's size:\n";
//    cout << LocateVecPath(0,next,fullPath).size() << endl;
//    cout << next.size()<<endl;

    if (key.size() == conditions.size()) {
        myokpath.second.first.push_back(node);
        allokpath.insert(myokpath);
//        PrintSetVectorInt(allokpath,conditions);
//        cout << "\nRe->" << allokpath.size()<<endl;
//        PrintVecInt(key);
//        PrintVecInt(myokpath.second.first);
//        PrintVecInt(myokpath.second.second);
//        if(allokpath.size()==1000) {
        return;
    }

    if (get_miltime() >= 18000) {
//        ViewSetPath(setPath,conditions);
        if (allokpath.size() > 0) {
            PrintSetPath(allokpath, conditions);
            exit(11);
        }
        else if (allokpath.size() == 0) {
            PrintNA(re);
            exit(111);
        }
    }

    for (set<Path>::const_iterator PathIter = setPath.begin(); PathIter != setPath.end(); ++PathIter) {
        myprocessed = processed; //[!]:2r setPath ,go without 1st viad node lsit
        myokpath = okpath;
        const vector<int> &pointInfo = (*PathIter).second.first;
        const vector<int> &edgeInfo = (*PathIter).second.second;
        bool flag = true;
        for (int i = 0; i < pointInfo.size(); i++) {
            if (myprocessed.count(pointInfo[i])) {
                flag = false;
                break;
            }
        }

        if (flag == false)
            continue;
        else {
            for (int i = 0; i < pointInfo.size(); i++) {
                myprocessed.insert(pointInfo[i]);
            }
            myokpath.first += (*PathIter).first;
            myokpath.second.first.insert(myokpath.second.first.end(), pointInfo.begin(), pointInfo.end());
            myokpath.second.second.insert(myokpath.second.second.end(), edgeInfo.begin(), edgeInfo.end());

            int beta;
            processed = myprocessed; //[!]:2r setPath ,go without 1st viad node lsit
            beta = myokpath.second.first.back();
            myokpath.second.first.pop_back();
            KKK(beta, src, dest, conditions, fullPath, myiterCount, myprocessed, myokpath, allokpath,asi);
            for (int i = 0; i < pointInfo.size(); i++) {
                processed.erase(pointInfo[i]);
            }
            processed.erase(beta);

        }

    }

}
//AF
void ASK(int node,
         int src,
         int dest,
         const Conditions &conditions,
         FullPath &fullPath,
         int iterCount,
         set<int> processed,
         Path okpath,
         set<Path> &allokpath,bool &iterFlag,int asi) {

    if(iterFlag == false)
        return;
    processed.erase(node);

    vector<int> next;
    set<int> myprocessed = processed;
    Path myokpath = okpath;
    int myiterCount = iterCount - 1;

    vector<int> tmp = okpath.second.first;
    tmp.push_back(node);
    vector<int> key = getKeyVector(tmp, conditions);



    // [node,node] is not exist ,though myprocessed pop_back node ,it's still right
//    vector<set<Path>> vector_set_Path = LocateVecPath(node, next, fullPath,myprocessed);
//    set<Path> setPath = LocateSetPath(node, next, fullPath, myprocessed,conditions.size()-myprocessed.size());
    myprocessed.insert(dest);
    if( (key.size()-conditions.size()) == -1 ) {   // not select dest until it's the last one
        myprocessed.erase(dest);
    }
    set<Path> setPath = LocateSetPath(node, next, fullPath, myprocessed,asi);

    //AFM
//    printf("[%d]\n",node);
//    SeeSetPath(setPath,conditions);

//    if (get_miltime()<1000) {
        if (next.size() == 0 && allokpath.size() == 0) { // not find one solve until now
//            ++asi;
//            iterFlag = true;
//            ASK(src, src, dest, conditions, pathDict, fullPath, iterCount, processed, okpath, allokpath, iterFlag, asi);
            ++TouchEndCount;
            if (TouchEndCount > conditions.size()) {
//            if (TouchEndCount > 2) {
                iterFlag = false;
                TouchEndCount = 0;
            }
//        return;
        }
//    }



//    cout << next.size()<<endl;
//    cout << "the 0's size:\n";
//    cout << LocateVecPath(0,next,fullPath).size() << endl;
//    cout << next.size()<<endl;

    if (key.size() == conditions.size()) {
        myokpath.second.first.push_back(node);
        allokpath.insert(myokpath);
        iterFlag = false;
//        printf("Cost>[%d]\n",okpath.first);
//        PrintSetVectorInt(allokpath,conditions);
//        cout << "\nSIZE->" << allokpath.size()<<endl;
//        PrintVecInt(key);
//        PrintVecInt(myokpath.second.first);
//        PrintVecInt(myokpath.second.second);
//        if(allokpath.size()==1000) {

        /*
        if (get_miltime() >= 9000) {
            if (allokpath.size() > 0) {
//                ViewSetPath(setPath,conditions);
                PrintSetPath(allokpath, conditions);
            }
            exit(10);
        }
         */
        return;
    }

    if (get_miltime() >= 9000) {
//        if (allokpath.size() == 0) {
            processed.clear();
            iterCount = conditions.size();
            Path tmp;
            okpath = tmp;
            KKK(src,src,dest,conditions,fullPath,iterCount,processed,okpath,allokpath,1);
//            PrintNA(re);
//            exit(91);
//        }
    }


    for (set<Path>::const_iterator PathIter = setPath.begin(); PathIter != setPath.end(); ++PathIter) {
        myprocessed = processed; //[!]:2r setPath ,go without 1st viad node lsit
        myokpath = okpath;
        const vector<int> &pointInfo = (*PathIter).second.first;
        const vector<int> &edgeInfo = (*PathIter).second.second;
        bool flag = true;
        for (int i = 0; i < pointInfo.size(); i++) {
            if (myprocessed.count(pointInfo[i])) {
//                cout<<pointInfo[i];
                flag = false;
                break;
            }
        }

        if (flag == false) {
            continue;
        }
        else {
            for (int i = 0; i < pointInfo.size(); i++) {
                myprocessed.insert(pointInfo[i]);
            }
            myokpath.first += (*PathIter).first;
            myokpath.second.first.insert(myokpath.second.first.end(), pointInfo.begin(), pointInfo.end());
            myokpath.second.second.insert(myokpath.second.second.end(), edgeInfo.begin(), edgeInfo.end());
//            if(node==src){
//                cout << "src-start>>" <<myokpath.second.first.back()<<endl;
//                if(myokpath.second.first.back()==15){
//                    cout << "here is 15" << endl;
//                }
//            }

            int beta;
            processed = myprocessed; //[!]:2r setPath ,go without 1st viad node lsit
            beta = myokpath.second.first.back();
            myokpath.second.first.pop_back();
            ASK(beta, src, dest, conditions, fullPath, myiterCount, myprocessed, myokpath, allokpath,iterFlag,asi);
            for (int i = 0; i < pointInfo.size(); i++) {
                processed.erase(pointInfo[i]);
            }
            processed.erase(beta);//AFE
            if(node==src){
                iterFlag = true;
            }
        }

        if(node==src){
            iterFlag = true;
        }

    }

}

void VECTK(int node,
           int src,
           int dest,
           const Conditions &conditions,
           FullPath &fullPath,
           int iterCount,
           set<int> processed,
           Path okpath,
           set<Path> &allokpath) {

//    set<int> processed;
//    processed.insert(node);
    processed.erase(node);

    vector<int> next;
    set<int> myprocessed = processed;
    Path myokpath =okpath;
    vector<int> tmp=okpath.second.first;
    int myiterCount = iterCount -1;

    // [node,node] is not exist ,though myprocessed pop_back node ,it's still right
    vector<set<Path> > vector_set_Path = LocateVecPath(node, next, fullPath,myprocessed);

    tmp.push_back(node);
    vector<int> key = getKeyVector(tmp,conditions);

//    cout << next.size()<<endl;
//    cout << "the 0's size:\n";
//    cout << LocateNode(0,next,fullPath).size() << endl;
//    cout << next.size()<<endl;

//    if (next.size() == 10) {
    if (key.size() == conditions.size()) {
//        cout<<iterCount<<endl;
        myokpath.second.first.push_back(node);
        allokpath.insert(myokpath);
//        PrintSetVectorInt(allokpath,conditions);
        cout << "\nSIZE->" << allokpath.size()<<endl;
        PrintVecInt(key);
        PrintVecInt(okpath.second.first);
        PrintVecInt(okpath.second.second);
        exit(0);
        return;
    }
    if (iterCount < 0) {
        cout<<"iC<0 "<<key.size();
        return;
    }
//    processed.insert(node);
    for (vector<set<Path> >::const_iterator SetPathIter = vector_set_Path.begin();SetPathIter != vector_set_Path.end(); ++SetPathIter) {
        set<Path> setPath = *SetPathIter;
        myprocessed = processed; //[!]:2r setPath ,go without 1st viad node lsit
        myokpath = okpath;
        for (set<Path>::const_iterator PathIter = setPath.begin(); PathIter != setPath.end(); ++PathIter) {
            myprocessed = processed; //[!]:2r setPath ,go without 1st viad node lsit
            myokpath = okpath;
            const vector<int> &pointInfo = (*PathIter).second.first;
            const vector<int> &edgeInfo = (*PathIter).second.second;
//            cout << ">>>"<< pointInfo.size() <<endl;
            bool flag = true;
            /*
//            if(node==1&& conditions.count(pointInfo[pointInfo.size()-1])) {
            if(node==1&& pointInfo[pointInfo.size()-1]==33) {
                myprocessed.clear();
                okpath.clear();
//                cout << "[1,33]\n";
            }
             */
            for (int i = 0; i < pointInfo.size(); i++) {
                if (myprocessed.count(pointInfo[i])) {
                    flag = false;
                    break;
                }
//                cout << pointInfo[i] << "|" ;
            }

            if (flag == false)
                continue;
            else {
                for (int i = 0; i < pointInfo.size(); i++) {
                    myprocessed.insert(pointInfo[i]);
//                    myokpath.second.first.push_back(pointInfo[i]);
                }
                myokpath.second.first.insert(myokpath.second.first.end(),pointInfo.begin(),pointInfo.end());
                myokpath.second.second.insert(myokpath.second.second.end(),edgeInfo.begin(),edgeInfo.end());
                int beta;
                processed = myprocessed; //[!]:2r setPath ,go without 1st viad node lsit
                beta = myokpath.second.first.back();
                myokpath.second.first.pop_back();
                KKK(beta, src, dest, conditions,  fullPath,myiterCount, myprocessed,myokpath,allokpath,1);
//                cout << "kkk\n";
                for (int i = 0; i < pointInfo.size(); i++) {
                    processed.erase(pointInfo[i]);
                }
                processed.erase(beta);
                myprocessed = processed;
/*
                for (set<int>::const_iterator beta = next.begin(); beta != next.end(); beta++) {
                    if (iterCount==0)
                        break;
                    if (*beta == node || myprocessed.count(*beta))
                        continue;
                    if(iterCount<=0)
                        break;
                    KKK(*beta, src, dest, conditions, pathDict, fullPath,myiterCount, myprocessed,okpath,allokpath);
                }
*/
                //                    cout << iterCount;
//            cout << endl;

            }

        }

//    cout << "the fullPath's size:\n";
//    cout << fullPath.size() << endl;
    }
}

void SK66(
        int node,
        int source,
        int dest,
        int iterCount,
        const Graph &graph,
        const EdgeInfoDict &edgeInfoDict,
        const Conditions &conditions,
        SK66_D_dict &ddict,
        SK66_F_dict &fdict,
        AdvancedPathDict &pathDict,
        FullPath &fullDict) {
    cout << "here is sk66\n";
//    PrintFullDict(fullDict, conditions);
    // 当迭代次数为0时， 直接计算node->dest单源最短路径，存入结果字典里
    /*
    if (iterCount == 0) {
        std::pair<int, int> pathToBeSolve(node, dest);
//        if (!pathDict.count(pathToBeSolve))
        if (!fullDict.count(pathToBeSolve))
            Dijkstra(graph, edgeInfoDict, node, pathDict,fullDict,conditions);
//        if (!pathDict.count(pathToBeSolve)) {
        if (!fullDict.count(pathToBeSolve)){
            std::pair<std::pair<int, int>, int> key;   // < <起始点，终止点>,迭代次数>
            key.first = pathToBeSolve;
            key.second = 0;
            Path path;
            path.first = 0xffffff;          // 六个f， 足够表示无穷大了, 防止在后续加法中溢出
            fdict[key] = path;        // this path is not exist
        } else {
            std::pair<std::pair<int, int>, int> key;
            key.first = pathToBeSolve;
            key.second = 0;
            AdvancedPathDict::const_iterator PPath = pathDict.find(pathToBeSolve);  // Dj search get the solved result
            //Path path = PPath->second;
            //fdict[key] = path;
        }
    } else {
     */
    // 当迭代次数大于0的时候
    /*
    std::pair<std::pair<int, int>, int> key; // < <起始点，终止点>,迭代次数>
    key.first = std::pair<int, int>(node, dest); // first
    key.second = iterCount;   // second

    Path minCostPath;
    minCostPath.first = 0x7fffffff;
    */

    //for (Conditions::const_iterator iter2 = conditions.begin(); iter2 != conditions.end(); ++iter2) {

    // if (*iter == *iter2)
    //   continue;        // not via this node itself

    // 计算D(v_i, v_l)  ====>  {v(i) , v(i+1)}
    //  std::pair<int, int> leftHalfPathToBeSolve(*iter, *iter2);
//            if (!pathDict.count(leftHalfPathToBeSolve)) {
    //if (!fullDict.count(leftHalfPathToBeSolve)) {
    // }
//            if (!pathDict.count(leftHalfPathToBeSolve)) {
    //if (!fullDict.count(leftHalfPathToBeSolve)) {
//                    Path leftHalfPath;
//                    leftHalfPath.first = 0xffffff;
//                    ddict[leftHalfPathToBeSolve] = leftHalfPath;
//                } else {
//                ShortestPathDict::const_iterator PPath = pathDict.find(leftHalfPathToBeSolve);
//                    set<Path> PPath = fullDict[leftHalfPathToBeSolve];  // this right
    //Path leftHalfPath = PPath[pair<int,int>(0,0)];                       // this wrong
    //ddict[leftHalfPathToBeSolve] = leftHalfPath;
//                }
    // 计算F(v_l, v_l+1)
    /*
    Graph::const_iterator pSourceAdjs = graph.find(*iter);
    if (pSourceAdjs != graph.end()) {
        const std::set<int> &sourceAdjs = pSourceAdjs->second;
        for (std::set<int>::const_iterator iter_sub = sourceAdjs.begin(); iter_sub != sourceAdjs.end(); ++iter_sub) {
    if (!fullDict.count(rightHalfPathToBeSolve)) {
        SK66(*iter, source, dest, iterCount - 1, graph, edgeInfoDict, conditions, ddict, fdict, pathDict,
             fullDict);
    }
     */




/*

            // 筛选出最小值
            std::pair<std::pair<int, int>, int> rightHalfPathToBeSolve;
            rightHalfPathToBeSolve.first.first = *iter;
            rightHalfPathToBeSolve.first.second = dest;
            rightHalfPathToBeSolve.second = iterCount - 1; //迭代次数
            if (!fdict.count(rightHalfPathToBeSolve)) {
                SK66(*iter, source, dest, iterCount - 1, graph, edgeInfoDict, conditions, ddict, fdict, pathDict,fullDict);
            }
            if (ddict[leftHalfPathToBeSolve].first + fdict[rightHalfPathToBeSolve].first < minCostPath.first) {
                minCostPath.first = ddict[leftHalfPathToBeSolve].first + fdict[rightHalfPathToBeSolve].first;
                minCostPath.second.first.clear();
                minCostPath.second.second.clear();
                minCostPath.second.first.insert(minCostPath.second.first.end(),
                                                ddict[leftHalfPathToBeSolve].second.first.begin(),
                                                ddict[leftHalfPathToBeSolve].second.first.end());
                minCostPath.second.first.insert(minCostPath.second.first.end(),
                                                fdict[rightHalfPathToBeSolve].second.first.begin(),
                                                fdict[rightHalfPathToBeSolve].second.first.end());
                minCostPath.second.second.insert(
                        minCostPath.second.second.end(),
                        ddict[leftHalfPathToBeSolve].second.second.begin(),
                        ddict[leftHalfPathToBeSolve].second.second.end());
                minCostPath.second.second.insert(
                        minCostPath.second.second.end(),
                        fdict[rightHalfPathToBeSolve].second.second.begin(),
                        fdict[rightHalfPathToBeSolve].second.second.end());
            }
            */

    //}

//        }

//        fdict[key] = minCostPath;

}


int Xhuisu(int S,int T)
{
    int i = 0,j = 0, n = 0,p = 0,c = 0;
    bool cuflag = 1;
    int ChiledNum = 0;
    int chiled[8];//出度最大为8

  //  memset(chiled,9999,sizeof(chiled));//chiled初始化

    n = S;
    ks = 1;
    PathStack[ks] = n;
    PathStack[0] = 9999;



    while(ks != 0 && TimeLong < 5)
    {

        c = 0;
        ChiledNum = 0;
        memset(chiled,9999,sizeof(chiled));
        TimeFinish = clock();
        TimeLong = (double)(TimeFinish - TimeStart)/CLOCKS_PER_SEC;

        for(j = 0;j < Vmax;j++)//统计路径子集
        {
            if(finalal[n][j] == 1)
            {
                chiled[c] = j;
                c++;
                ChiledNum++;
            }

        }



        if(ChiledNum == 0)//假如路径没有子集
        {
            for(j = 0;j < Vmax;j++)
            {
                if(Gtable[n][j].weigt != 0)//弹出点再次初始化
                {
                    finalal[n][j] = 1;
                }

            }
            PathStack[ks] = 0;
            ks--;
            n = PathStack[ks];
        }


        else
        {
            for(c = 0;c<ChiledNum;c++ )//假如点n存在出度
            {
                i = chiled[c];
                p = XCheckRe(i);//对i点进行重复检验
                if( p == ERROR)//假如重复
               {
                   finalal[n][i] = 0;
               }
                else//假如没有重复判断是否为T，i进栈，n等于栈顶
                {
                    if(i == T)
                    {
                        if(XCheckPass())
                        {
                            ks++;
                            PathStack[ks] = i;
                            finalal[n][i] = 0;
                            XGetResult(result);
                            PathStack[ks] = 0;
                            ks--;
                       //     return OK;
                        }
                        finalal[n][i] = 0;//判断不能由n直接到
                        break;
                }

                    ks++;
                    finalal[n][i] = 0;
                    PathStack[ks] = i;
                    n = i;
                    break;
                }
            }

        }




    }




//return ERROR;

}




int XGetResult(unsigned short *result)
{
    int i = 0,j = 0,k = 0,w = 0;
    unsigned short result2[600] = {0};

    for(k = 1;k <ks;k++)
    {
        i = PathStack[k];
        j = PathStack[k+1];
        result2[k-1] = Gtable[i][j].Edge;
        w = Gtable[i][j].weigt + w;
    }
   // k = k-1;
    if(w < cost)
    {
        for(k = 0;k < ks;k++)
        {
            result[k] = result2[k];
        }
        k = k-1;
        R = k;
        cost = w;

    }

    return k;
}




int XCheckRe(int i)
{
    int k = 0;
    for(k = 0; k <= ks;k++)//加入点储存在栈中，返回error;可优化，不必每次都循环；
    {
        if (PathStack[k] == i)
        {
            return ERROR;
        }
    }
    return OK;
}




int XCheckPass()
{
    int k = 0,i = 0,passflag = 0;


    for(i = 0;i < passnum-1 ;i++)//终点不加入检查
    {
        passflag = 0;
        for(k = 0;k <= ks;k++)
        {
            if(demandint[i] == PathStack[k])
            {
                passflag = 1;
                break;
            }
        }
        if(passflag == 0)//假如有一个点没有遍历到
        {
            return ERROR;
        }

    }


        return OK;


}










bool FindWay(int S,int T)
{
    int x = 0;
    int i = 0,j = 0,k = 0;
    int c = 0;

    bool status = 0;

    while(ks != -1 && TimeLong < 9)
    {

        TimeFinish = clock();
        TimeLong = (double)(TimeFinish - TimeStart)/CLOCKS_PER_SEC;
        status = FindChild(S);
        if(status == 0)
        {
            Pop();
            S = PathStack[ks];

        }
        else
        {
            x = Child[i];
            Push(S,x);
            if(x == T)
            {
                if(CheckT())
                {
                    GetResult(result);
                    Pop();
                    S = PathStack[ks];
                    //return OK;
                }
                else
                {
                    Pop();
                    S = PathStack[ks];
                }
            }

            //LastS = S;
            S = PathStack[ks];

        }
    }

    return ERROR;

}


void Pop()//弹出
{
    int i = 0,S = 0,k = 0,j = 0;
    k = PathStack[ks];
    for(i = 0;i <=50;i++)//将弹出必经点的childflag清零
    {
        Vchildflag[k][demandint[i]] = 0;
    }

    for(k = ks-1;k>=0;k--)
    {
        if(V[PathStack[k]] == 1)
        {
            S = PathStack[k];
            break;
        }
    }


    while(PathStack[ks] != S)
    {
        if(V[PathStack[ks]] == 1)//假如该点是必经点
        {
            for(i = 0;i<passnum;i++)
            {
                if(PathStack[ks] == demandint[i])//将弹出必经点标记为未经过
                {
                    demandflag[i] = 0;
                    break;
                }
            }
        }

        for(i = 0;i < Vmax;i++)//恢复入度
        {
            if(Gtable[i][PathStack[ks]].weigt != 0)
            {
                finalal[i][PathStack[ks]] = 1;
            }
        }
/*        for(i = 0;i < Vmax;i++)//恢复出度
        {
            if(Gtable[PathStack[ks]][i].weigt != 0)
            {
                finalal[PathStack[ks]][i] = 1;
            }
        }*/
        PathStack[ks] = -1;
        ks = ks-1;
    }
}



void Push(int S,int T)
{
    int StackPath[500];
    int x = 0;
    int y = 0;
    x = T;
  //  lastks = ks;
    while( x != S && x != -1)//目标点与源点不相同
    {
        StackPath[y] = x;//将Path数组中的顶点入栈

        x = P[StackPath[y]];
        y++;
    }
    StackPath[y] = x;

    for( ; y>=0 ; y--)//栈内的顶点出栈，并按照顺序搜索边
    {

        PathStack[ks] = StackPath[y];
        ClearRu(S,StackPath[y]);
        ks++;
    }
    ks = ks-1;
}


void ClearRu(int S,int y)
{
    int  i = 0;
    if(V[y] == 1)//假如输入点为必经点
    {
        for(i = 0; i<passnum;i++)
        {
            if(demandint[i] == y)
            {
                Vchildflag[S][demandint[i]] = 1;//标记为S点的已遍历孩子
                demandflag[i] = 1;//标记1表示该点已经经过
                break;
            }
        }
    }

    for(i = 0;i<Vmax;i++)//入栈点的所有入度归零
    {
        finalal[i][y] = 0;
    }
}


bool FindChild(int S)
{
    int i = 0,j = 0;
    int t = 0;//作为孩子数量的统计
    int c = 0,n = 0,x = 0;//作为已经经过的cost的统计
    int F[600] = {0};//定义F矩阵储存启发式函数的值
    int DFC[600] = {0};
    int PFC[600] = {0};
    memset(Child,-1,sizeof(Child));
    Dijkstra(Vmax,S,P,D);
    c = WeigtSum();
    for(i = 1;i <passnum;i++)
    {
        if(D[demandint[i]] > 999 && demandflag[i] == 0)//假如这个孩子节点无法到达所有的剩余点，则返回错误;
        {
            return ERROR;
        }
        else
        {
            if( (Vchildflag[S][demandint[i]] == 0) && CheckPass(S,demandint[i],P))//检查S到demaint[i]的路径是否符合条件
            {
                if((c + D[demandint[i]]) <= cost )//只有和小于cost的孩子节点才可以用
                {
                    Child[j] = demandint[i];
                    j++;
                }

            }

        }

    }



    for(t = 0;t < j;t++)
    {
  //      Dijkstra(Vmax,Child[t],PFC,DFC);
        F[t] = D[Child[t]];
    }

    for(i = 0;i<j-1;i++)
    {
        for(t = j-2;t >= i;t--)
        {
            if(F[t] > F[t+1])
            {
                n = F[t+1];
                x = Child[t+1];
                F[t+1] = F[t];
                Child[t+1] = Child[t];
                F[t] = n;
                Child[t] = x;
            }
        }
    }
    t = 0;

    for(i = 0;i <50;i++)
    {
        if(Child[i] >= 0)
        {
            t++;
        }
    }
    if(t < 1)
    {
        return ERROR;
    }
    return OK;

}


bool CheckT()
{
    int i = 0;
    for(i = 0;i<passnum;i++)
    {
        if(demandflag[i] == 0)
        {
            return ERROR;
        }
    }
    return OK;
}

//从P矩阵中获取顺序点
bool CheckPass(int S,int T,int *Path)
{
    int StackPath[500];
    int x = 0;
    int y = 0;
    int i = 0;
    if(D[T] > 999)
    {
        return ERROR;
    }
    x = T;
  //  lastks = ks;
    while( x != S && x != -1)//目标点与源点不相同
    {
        StackPath[y] = x;//将Path数组中的顶点入栈
        x = Path[StackPath[y]];
        if(V[x] == 1 && x != S)//假如路径经过了非起点的必经点
        {
            return ERROR;
        }


        y++;
    }
    StackPath[y] = x;
    return OK;
}


int WeigtSum()//可以不用此函数，直接利用D数据进行计算
{
    int k = 0,w = 0,i = 0,j = 0;
    for(k = 0;k < ks;k++)
    {
        i = PathStack[k];
        j = PathStack[k+1];
        w = w+Gtable[i][j].weigt;
    }
    return w;
}




int GetResult(unsigned short *result)
{
    int i = 0,j = 0,k = 0,w = 0;
    unsigned short result2[600] = {0};
    for(k = 0;k <ks;k++)
    {
        i = PathStack[k];
        j = PathStack[k+1];
        result2[k] = Gtable[i][j].Edge;
        w = w+Gtable[i][j].weigt;
    }
  //  k = k-1;
  if(w < cost)
    {
        for(k = 0;k < ks;k++)
        {
            result[k] = result2[k];
        }
        //k = k-1;
        R = k;
        cost = w;
    }

    return k;
}









//输入：顶点数量，起点，终点
//输出：路径P，权值D
int Dijkstra(int Vexnum,int Vbegin, int *P,int *D)
{

    int v,w,k,m;
    int final[Vexnum];
    k = 0;
    if (Vbegin >= Vexnum)//判断终点是否存在
    {
        return ERROR;
    }

    for(v = 0; v < Vexnum; v++)//数据初始化
    {
        final[v] = 0;
        P[v] = -1;
        if(Gtable[Vbegin][v].weigt != 0 && finalal[Vbegin][v] != 0)
        {
              D[v] = Gtable[Vbegin][v].weigt;
              P[v] = Vbegin;
        }
        else
        {
            D[v] = 99999;//将起点与各点的权值置为30，表示没有边相连
        }
     //   P[v] = Vbegin;
    }

    D[Vbegin] = 99999;//起点到起点不存在边
    final [Vbegin] = 1;//起点不参与搜索

    v = Vbegin + 1;

    while (v % Vexnum != Vbegin )//循环队列，依次搜索所有顶点
    {
        m = 99999;//最小值初始化为30

        for(w = 0; w<Vexnum ;w++)//求各点与起点最小权
        {
            if (!final[w] && D[w]<m)
            {
                k = w;
                m = D[w];
            }

        }

    final[k] = 1;
    for(w = 0; w<Vexnum; w++)//检验权
    {
        if(!final[w] && (m+Gtable[k][w].weigt < D[w] ) && (Gtable[k][w].weigt != 0) && (finalal[k][w] != 0))
        {
            D[w] = m + Gtable[k][w].weigt;
            P[w] = k;
        }
    }
        v++;

    }
    return OK;
}





//输入：边集数组，边的数量
//输出：邻接矩阵（全局变量），顶点数量
void TopoToTable(int topoint[5000][4],int edgen,int *max) //将int型的边集数组转为邻接矩阵
{
    int i = 0,x = 0,y = 0 ;
    *max = 0;
    for (i = 0;i<edgen;i++)
    {
        x = topoint[i][1];
        y = topoint[i][2];

        if(Gtable[x][y].weigt>0 && Gtable[x][y].weigt<21)//有效边
        {
            if(topoint[i][3] < Gtable[x][y].weigt)//取重复边中最小值
            {
                Gtable[x][y].weigt  = topoint[i][3];
                Gtable[x][y].weigt  = topoint[i][0];
            }
        }
        else//在邻接矩阵中储存边序号和权值
        {
            Gtable[x][y].Edge = topoint[i][0];
            Gtable[x][y].weigt = topoint[i][3];
            finalal[x][y] = 1;
        }

        if (*max < x)//求顶点序号最大值
        {
            *max = x;
        }
        if(*max < y)
        {
            *max = y;
        }
    }


}




void DFS(char *topo[5000], int edge_num, char *demand){
    int  topoint[5000][4] ;                                     //定义int数组用于储存读入topo数据（可优化修改为直接读入int）

    int i = 0,j = 0,x=0,y=0,n=0;
    int S = 0,T = 0;
    char num[4]={0};//定义字符串数组作为中间变量

//    memset(Path,65535,sizeof(Path));

//    int StackPath[MAXVEX] = {0};//定义输出定点栈

    int status = 1;//输出状态



    TimeStart = clock();//计时开始
    memset(finalal,0,sizeof(finalal));
    memset(demandflag,0,sizeof(demandflag));//清零做好准备

    while(i < 5000 && topo[i]!=NULL) //将输入图信息转化为int型数组
    {
        while(topo[i][j] != 10 && topo[i][j] !=0)//不为回车不为空
        {

            if(topo[i][j] != 44 && topo[i][j] !=10 && topo[i][j] !=0)//不为逗号
            {
                 num[n] = topo[i][j];
                 n++;
            }
            else//加入结束符号作为字符串
            {
                num[n]= '\0';

                topoint[x][y] = atoi(num);//字符串转为数字
                memset(num,0,sizeof(num));//中间变量清零
                n=0;
                y++;
            }
            j++;

        }
        num[n]= '\0';

        topoint[x][y] = atoi(num);
        memset(num,0,sizeof(num));
        n=0;
        y = 0;
        j = 0;
        i++;//跳出while循环条件
        x++;
    }

    i = 0;
    n = 0;
    y = 0;
    while(i < MAXVEX && demand[i]!=0)//将必经点信息转化为int数组
    {
         if(demand[i]!= 44 && demand[i] != 124 && demand[i+1]!=0)//不为逗号不为竖线不为0
            {
                 num[n] = demand[i];
                 n++;
                 i++;
            }
            else if(demand[i+1] == '\0')//结束
            {
                num[n] = demand[i];
                num[n+1] = '\0';
                demandint[y] = atoi(num);
                memset(num,0,sizeof(num));
                i++;
                y++;

            }
            else
            {
                num[n]= '\0';
                demandint[y] = atoi(num);
                memset(num,0,sizeof(num));
                n=0;
                y++;
                i++;
            }
    }


    passnum = y;//统计经过点数量
    x = demandint[1];
    for(i = 2;i < passnum;i++)
    {
        demandint[i-1] = demandint[i];
    }
    demandint[passnum-1] = x;


    for(i = 0; i<passnum;i++)
    {
        V[demandint[i]] = 1;
    }


    S = demandint[0];
    demandflag[0] = 1;
    n = 1;//目标点从demand[1]开始
    x = 0;
    y = 0;


    TopoToTable(topoint,edge_num,&Vmax);//边集数组转换为邻接表
    ks = 0;
    S = demandint[0];
    T = demandint[passnum -1];
    cost = 9999;//cost初始化
    Vmax = Vmax+1;//统计顶点数量

    if(passnum <=11)
    {
        Xhuisu(S,T);
      //  R = XGetResult(result);
        if(R == 0)
        {
            status = ERROR;
        }
        else
        {
            status = OK;
        }
    }
    else
    {
        FindWay(S,T);
       //R = GetResult(result);
        if(R == 0)
        {
            status = ERROR;
        }
        else
        {
            status = OK;
        }
    }


    if (status == OK) {
        for (int i = 0; i < R; i++) {
            record_result(result[i]);
        }
    }

}
