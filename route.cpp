#include "route.h"
#include "lib/lib_record.h"
#include <stdio.h>
#include <map>
#include <set>
#include <vector>
#include <iostream>

using namespace std;

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

typedef map<pair<int,int>,set<pair<int,vector<int>>>> AdvancedPathDict;

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

//--------------------------------------------------------------------------------------------------------数据输入模块函数
int ReadANumberFromStr(char *, int &);                          // 从字符流中读取一个数字
void ReadGraphData(char **, Graph &, EdgeInfoDict &);           // 读取图信息
void ReadConditionsData(char *, int &, int &, Conditions &);    // 读取约束条件信息
//--------------------------------------------------------------------------------------------------------测试函数
void PrintGraph(const Graph &, const EdgeInfoDict &);   //向控制台输出图信息
void PrintConditions(int, int, const Conditions &);  //向控制台输出约束条件信息
void PrintShortestPathDict(const ShortestPathDict &);  //向控制台输出最短路径字典中的信息
void PrintFullDict(const FullPath &,const Conditions &);
//--------------------------------------------------------------------------------------------------------算法函数
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

void KKK(
        int node,
        int src,
        int dest,
        const Conditions &,
        AdvancedPathDict &,
        FullPath &,
        int ,
        set<int>,
        vector<int> okpath,
        set<vector<int>> &allokpath);

vector<set<Path>> LocateNode(int node,set<int> &dest,const FullPath  &fullPath,const set<int> &processed){
    vector<set<Path>> sum;
    for (FullPath::const_iterator iter = fullPath.begin(); iter != fullPath.end(); ++iter) {
//        cout <<"[XXXX]\n";
//        printf("[%d,%d]\n",(iter->first).first,(iter->first).second );
//        cout<< (iter->first).first << (iter->first).second << endl;
        if ( node == (iter->first).first && !processed.count((iter->first).second)){
            dest.insert((iter->first).second);
            sum.push_back(iter->second);
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
    vector<int> okpath;
    set<vector<int>> allokpath;

//    std::set<int> without;
//    without.insert(1);
//    without.insert(2);




//    conditions.insert(1); // insert the src node
//    conditions.insert(429); // dst should not act as src node

    for (Conditions::const_iterator iter = conditions.begin(); iter != conditions.end(); ++iter) {
        Dijkstra(graph, edgeInfoDict, *iter, pathDict, fullDict, conditions);
    }
//    PrintFullDict(fullDict, conditions);
//    return;
//    processed.insert(0);   // <--  0|129|220|...|  start with 0
    KKK(1,1,429,conditions,pathDict,fullDict,iterCount,processed,okpath,allokpath);
    cout << "the ok path size>>>" <<allokpath.size() << endl;

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

//--------------------------------------------------------------------------------------------------------测试函数实现
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
                    pathDict[pair<int,int>(source,*iter)].insert(pair<int,vector<int>>(xedgeInfo.second,xpath.second.first));
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
            if(*iter==source)
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
                        pair<int,vector<int>>(candidate.pathCost,candidate.nodePath)
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

void KKK(int node,
         int src,
         int dest,
         const Conditions &conditions,
         AdvancedPathDict &pathDict,
         FullPath &fullPath,
         int iterCount,
         set<int> processed,
         vector<int> okpath,
         set<vector<int>> &allokpath) {

//    set<int> processed;
//    processed.insert(node);
    processed.erase(node);

    set<int> next;
    set<int> myprocessed = processed;
    int myiterCount = iterCount -1;

    vector<set<Path>> vector_set_Path = LocateNode(node, next, fullPath,myprocessed);

//    cout << next.size()<<endl;
//    cout << "the 0's size:\n";
//    cout << LocateNode(0,next,fullPath).size() << endl;
//    cout << next.size()<<endl;

    if (next.size() == 0) {
//        cout<<iterCount<<endl;
        allokpath.insert(okpath);
        return;
    }
    if (iterCount == -1) {
        cout<<iterCount;
        return;
    }
//    processed.insert(node);
    for (vector<set<Path>>::const_iterator SetPathIter = vector_set_Path.begin();SetPathIter != vector_set_Path.end(); ++SetPathIter) {
        set<Path> setPath = *SetPathIter;
        for (set<Path>::const_iterator PathIter = setPath.begin(); PathIter != setPath.end(); ++PathIter) {
            const vector<int> &pointInfo = (*PathIter).second.first;
//            cout << ">>>"<< pointInfo.size() <<endl;
            bool flag = true;
            if(node==1&& conditions.count(pointInfo[pointInfo.size()-1])) {
                myprocessed.clear();
//                cout << "[1,33]\n";
            }
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
                    okpath.push_back(pointInfo[i]);
                }
                int beta;
                beta = okpath.back();
                if(beta==160)
                    cout<< "debug point\n";
                okpath.pop_back();
                KKK(beta, src, dest, conditions, pathDict, fullPath,myiterCount, myprocessed,okpath,allokpath);
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
