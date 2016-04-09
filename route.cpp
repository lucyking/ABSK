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

using namespace std;

extern char *re;
int TouchEndCount =0;
int asi = 0;
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
        AdvancedPathDict &,
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
        AdvancedPathDict &,
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
    if(conditions.size()<8){
//        ASK(source, source, dest, conditions, pathDict, fullDict, iterCount, processed, okpath, allokpath, iterFlag,10); //MMM
        KKK(source,source,dest,conditions,pathDict,fullDict,iterCount,processed,okpath,allokpath,100);
    }
    else {
        bool iterFlag = true;
        ASK(source, source, dest, conditions, pathDict, fullDict, iterCount, processed, okpath, allokpath, iterFlag,10); //MMM
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

    printf("路径花费 == %d\n", ansCost);
    printf("共经过了「%ld」个结点： ", pointPath.size());
    for (std::vector<int>::const_iterator iter = pointPath.begin(); iter != pointPath.end(); ++iter)
        printf("%d|", (*iter));
    printf("\n");
    printf("共经过了「%ld」条边: ", edgePath.size());
    for (std::vector<int>::const_iterator iter = edgePath.begin(); iter != edgePath.end(); ++iter)
        printf("%d|", (*iter));
    printf("\n");
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
         AdvancedPathDict &pathDict,
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
        if (get_miltime() >= 9800) {
            if (allokpath.size() > 0) {
//                ViewSetPath(setPath,conditions);
                PrintSetPath(allokpath, conditions);
            }
            exit(11);
        }
        return;
    }

    if (get_miltime() >= 9800) {
//        ViewSetPath(setPath,conditions);
        if (allokpath.size() == 0) {
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
            KKK(beta, src, dest, conditions, pathDict, fullPath, myiterCount, myprocessed, myokpath, allokpath,asi);
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
         AdvancedPathDict &pathDict,
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

    if (get_miltime()<1000) {
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
    }



//    cout << next.size()<<endl;
//    cout << "the 0's size:\n";
//    cout << LocateVecPath(0,next,fullPath).size() << endl;
//    cout << next.size()<<endl;

    if (key.size() == conditions.size()) {
        myokpath.second.first.push_back(node);
        allokpath.insert(myokpath);
//        printf("Cost>[%d]\n",okpath.first);
//        PrintSetVectorInt(allokpath,conditions);
//        cout << "\nSIZE->" << allokpath.size()<<endl;
//        PrintVecInt(key);
//        PrintVecInt(myokpath.second.first);
//        PrintVecInt(myokpath.second.second);
//        if(allokpath.size()==1000) {

        if (get_miltime() >= 1000) {
            if (allokpath.size() > 0) {
//                ViewSetPath(setPath,conditions);
                PrintSetPath(allokpath, conditions);
            }
            exit(10);
        }
        return;
    }

    if (get_miltime() >= 1000) {
        if (allokpath.size() == 0) {
            processed.clear();
            iterCount = conditions.size();
            Path tmp;
            okpath = tmp;
            KKK(src,src,dest,conditions,pathDict,fullPath,iterCount,processed,okpath,allokpath,1);
//            PrintNA(re);
//            exit(91);
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
            ASK(beta, src, dest, conditions, pathDict, fullPath, myiterCount, myprocessed, myokpath, allokpath,iterFlag,asi);
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
           AdvancedPathDict &pathDict,
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
                KKK(beta, src, dest, conditions, pathDict, fullPath,myiterCount, myprocessed,myokpath,allokpath,1);
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