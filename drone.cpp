// PROJECT IDENTIFIER: 1761414855B69983BD8035097EFBD312EB0527F0
#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <map>
#include <limits>
#include <math.h>


//DO NOT INLUDE??
#include <iomanip>


using namespace std;

class drone{
    private:
        struct Options {
            bool isMST = false;
            bool isFASTTSP = false;
            bool isOPTTSP = false;
        };

         struct vertexCharact {
            bool known = false;
            double minimEdgeWeight = numeric_limits<double>::infinity();
            size_t connectedTo;
        };

        struct Vertex {
            int32_t x;
            int32_t y;
            char campus;
            // vertexCharact vCharact;
        };

        vector<Vertex> map;
        vector<vertexCharact> mapVChar;
        Options options;
        bool isDone = false;
        

            
        vector<uint32_t> path; // currentPath
        // vector<uint32_t> currentPath;
        double currentLen = numeric_limits<double>::infinity();
        double bestPathLengthSeen;
        vector<uint32_t> bestPathSeen;

        //   void printBC(vector<uint32_t> finalPath){
        //     double totalLength = 0.0;
        //     for(size_t i = 0; i < finalPath.size()-1; i++){
        //         totalLength += sqrt(calculateDistance(map[finalPath[i]], map[finalPath[i+1]]));
        //     }
        //     cout << totalLength << "\n";
            
        //     for(size_t i = 0; i < finalPath.size(); i++){
        //         cout << finalPath[i] << " ";
        //     }

        // }

        void printC(){
            cout << bestPathLengthSeen << "\n";
            for(size_t i = 0; i < bestPathSeen.size(); i++){
                cout << bestPathSeen[i] << " ";
            }
        }

        void printB(double totalLength, vector<uint32_t> tour){
            // double totalLength = 0.0;
            // for(size_t i = 0; i < tour.size()-1; i++){
            //     totalLength += sqrt(calculateDistance(map[tour[i]], map[tour[i+1]]));
            // }
            // totalLength += sqrt(calculateDistance(map[tour[tour.size()-1]], map[tour[0]]));

            
                cout << totalLength << "\n";
                for(size_t i = 0; i < tour.size(); i++){
                    cout << tour[i] << " ";
                }
            cout << "\n";
            

        }

        

        void update(){
            double closingEdge = sqrt(calculateDistance(map[path[path.size()-1]], map[path[0]]));  
            //add closing edge
            currentLen += closingEdge;
            //check/update the bestsofar, check if its better and if so update it   
                    
            if(currentLen < bestPathLengthSeen){
                bestPathLengthSeen = currentLen;
                bestPathSeen = path;
                // cerr << "New Best Cost Achieved: "<< bestPathLengthSeen << "\n";
            }         
            //subtract that closing edge
            currentLen -= closingEdge;
            
        }

        bool promising(size_t permL){
            size_t k = path.size() - permL;

            // COMMENTED OUT FOR DEBUGGING REASONS
            if(k < 5){
                return true;
            }

            double possibleWeight = 0.0;
            possibleWeight += currentLen;

            
            size_t count = 0;
            size_t index = permL;
            vector<Vertex> MSTpath;
            vector<vertexCharact> mapVCharMST;
            MSTpath.reserve(k);
            mapVCharMST.reserve(k);

            //call MST (part a)
            while(count < k){
                vertexCharact v;
                MSTpath.push_back(map[path[index]]);
                mapVCharMST.push_back(v);
                count++;
                index++;
            }

            double mstWeight = MSTmode(MSTpath, mapVCharMST);
            possibleWeight += mstWeight;

            //connect closest unvisited to first node and last node
            double closestEdgeToFirst = numeric_limits<double>::infinity();
            

            //first perm node
            for(size_t j = 0; j < MSTpath.size(); j++){
                double edgeL = sqrt(calculateDistance(map[path[0]], MSTpath[j]));
                if(edgeL < closestEdgeToFirst){
                    closestEdgeToFirst = edgeL;
                    // closestVtoFirst =  MSTpath[j];
                }
            }
            possibleWeight += closestEdgeToFirst;

            //last perm node
            // if(permL-1 != 0){
            double closestEdgeToSecond = numeric_limits<double>::infinity();
            for(size_t j = 0; j < MSTpath.size(); j++){
                double edgeL = sqrt(calculateDistance(map[path[permL-1]], MSTpath[j]));
                if(edgeL < closestEdgeToSecond){
                    closestEdgeToSecond = edgeL;
                    // closestVtoLast =  MSTpath[j];
                }
            }
            // }
            possibleWeight += closestEdgeToSecond;


            // DEBUGGING
            // bool promise = (possibleWeight < bestPathLengthSeen);
            // for (size_t i = 0; i < path.size(); ++i)
            //     cerr << setw(2) << path[i] << ' ';
            // cerr << setw(4) << permL << setw(10) << currentLen;
            // cerr << setw(10) << closestEdgeToFirst << setw(10) << closestEdgeToSecond;
            // cerr << setw(10) << mstWeight << setw(10) << possibleWeight << "  " << promise << '\n';
            // END DEBUGGING

           
            

            if(possibleWeight < bestPathLengthSeen){
                return true;
            }

            return false;            
            
        }


        void genPerms(size_t permLength) {
            if (permLength == path.size()) { //if you have fixed all the vertices
            // permLength means how many vertices are fixed (ik the real cost / black edges). anything that is not fixed has yet to be fixed in a future call
            // in every future call permLength goes up by 1
            // path.size() - permlength = k : how many unvisited vertices there are
            // Do something with the path
            // call update() if its better (than the best so far)
            
            // when we update best so far, theres 2 parts: double for length of path and best path var (looks just like path) remembers what is the actual best path
                update();
                return;
            }  // if ..complete path

            if (!promising(permLength)) { // < 5
                return;
            }  // if ..not promising

            for (size_t i = permLength; i < path.size(); ++i) {
                swap(path[permLength], path[i]);
                double newEdgeLen = sqrt(calculateDistance(map[path[permLength-1]], map[path[permLength]]));
                currentLen += newEdgeLen;
                genPerms(permLength + 1); //make the next vertex in the path part of the fixed version
                currentLen -= newEdgeLen;
                swap(path[permLength], path[i]);
            }  // for ..unpermuted elements
        }  // genPerms()


    
        
        // double bestDistanceSeen;
        // vector<Vertex> bestPathSeen;
        // double runningTotal;

                

      

         double calculateDistance(Vertex a, Vertex b){
            if(options.isMST == true){
                if(a.campus != b.campus && a.campus != 'b' && b.campus != 'b'){
                    return numeric_limits<double>::infinity();
                }
            }
            double x1 = a.x;
            double y1 = a.y;
            double x2 = b.x;
            double y2 = b.y;
            double firstSquare = (x1 - x2) * (x1 - x2);
            double secondSquare = (y1 - y2) * (y1 - y2);
            
            return firstSquare + secondSquare; // did not use square root because of pro tip
        }

        double calculateTotalLengthFASTTSP(const vector<uint32_t>& tour){
            double totalLength = 0.0;
            for(size_t i = 0; i < tour.size() - 1; ++i){
                totalLength += sqrt(calculateDistance(map[tour[i]], map[tour[i+1]]));
            }
            return totalLength;

        }

        

        pair<double, vector<uint32_t>> FASTTSPmode(){     

            vector<uint32_t> tour;
            tour.reserve(map.size()+1);
            
            vector<bool> isPresentInTour(map.size(), false);            


            // size_t bestPos = 0; 

            // start tour with the first vertex
            tour.push_back(0);
            isPresentInTour[0] = true;



            tour.push_back(1);
            isPresentInTour[1] = true;
            tour.push_back(0);
        
            
            for(uint32_t i = 2; i < map.size(); i++){
                if(!isPresentInTour[i]){
                    double minCost = std::numeric_limits<double>::infinity();
                    size_t bestPos = 0;
                    for(size_t j = 0; j < tour.size()-1; j++){
                        double prevToNew = calculateDistance(map[tour[j]], map[i]);
                        double newToNext = calculateDistance(map[i], map[tour[j+1]]);
                        double prevToNext = calculateDistance(map[tour[j]], map[tour[j+1]]);
                        double potentialCost = prevToNew + newToNext - prevToNext;

                        if(potentialCost < minCost){
                            minCost = potentialCost;
                            bestPos = j+1;
                        }

                    }
                    auto it = tour.begin();
                    advance(it, bestPos);
                    tour.insert(it, i);
                    isPresentInTour[i] = true;
                }
                

            }

            // Output the final tour and its total length
            double totalLength = 0;
            
            

            // sqrt version
            // tour.pop_back();
            // for(size_t i = 0; i < tour.size()-1; i++){
            //     totalLength += sqrt(calculateDistance(map[tour[i]], map[tour[i+1]]));
            // }
            // totalLength += sqrt(calculateDistance(map[tour[tour.size()-1]], map[tour[0]]));

            totalLength = calculateTotalLengthFASTTSP(tour);
            tour.pop_back();                      
            
            return {totalLength, tour};
                        

        }

        void printMST(double weight){
            if (weight == 0.0){
                cerr << "Cannot construct MST\n";
                exit(1);
            }
            // double actualWeight = sqrt(weight);
            cout << weight << "\n";
            for(size_t currentV = 0; currentV < mapVChar.size(); currentV++){
                for(size_t i = 1; i < mapVChar.size(); i++){
                    if(mapVChar[i].connectedTo == currentV){
                        if (currentV < i){
                            cout << currentV << " " << i << "\n";
                        }
                        else{
                            cout << i << " " << currentV << "\n";
                        }
                    }
                }

            }
        }
        

        double MSTmode(vector<Vertex> &currentMap, vector<vertexCharact> &currentMapVChar){
            // pick root
            size_t currentVIndex = 0;
            currentMapVChar[0].minimEdgeWeight = 0;

            double currentDist = numeric_limits<double>::infinity();
            double weight = 0.0;

            for (size_t count = 0; count < currentMap.size(); count++){

                // find smallest false
                for (size_t i = 0; i < currentMap.size(); i++){
                    if (currentMapVChar[i].known == false){
                        if (currentMapVChar[i].minimEdgeWeight < currentDist){
                            currentVIndex = i;
                            currentDist = currentMapVChar[i].minimEdgeWeight;
                        }
                    }
                }

                // set smallest false to true
                currentMapVChar[currentVIndex].known = true;
                weight += currentMapVChar[currentVIndex].minimEdgeWeight;

                //update false neighbors
                for (size_t i = 0; i < currentMap.size(); i++){
                    if (currentMapVChar[i].known == false){
                        double distance = calculateDistance(currentMap[currentVIndex], currentMap[i]);
                        if(distance < currentMapVChar[i].minimEdgeWeight * currentMapVChar[i].minimEdgeWeight){
                            currentMapVChar[i].minimEdgeWeight = sqrt(distance);
                            currentMapVChar[i].connectedTo = currentVIndex;
                        }
                    }
                }
                currentDist = numeric_limits<double>::infinity();
            }

            // if(options.isMST == true){
            //     printMST(weight);
            // }

            return weight;
            

        }

        void OPTTSPmode(){
            
            auto [bestPathLSeen, bestPathSeen] = FASTTSPmode();
            path = bestPathSeen; 
            bestPathLengthSeen = bestPathLSeen;

            size_t permLen = 1;
            currentLen = 0;

            // DEBUGGING PURPOSES
            // bestPathLengthSeen = 336.74;
            // for(uint32_t i = 0; i <= 10; ++i) {
            //     path.push_back(i);
            // }
            // bestPathSeen = path;
            // END DEBUGGING            


            genPerms(permLen);
            // printB(bestPathLengthSeen, bestPathSeen);
            printC();
            //do not say best cost ever seen is inf
            
        }
        

    public:
        void getMode(int argc, char * argv[]){
            opterr = false; 
            int choice;
            int index = 0;
            option long_options[] = {
                { "mode", required_argument, nullptr, 'm'},
                { "help", no_argument, nullptr, 'h' },
                { nullptr, 0, nullptr, '\0' },
            };  

            while ((choice = getopt_long(argc, argv, "m:h", long_options, &index)) != -1) {
                switch (choice) {
                    case 'm' : { 
                        string arg{optarg};
                        if(arg == "MST"){
                            options.isMST = true;
                        }
                        else if(arg == "FASTTSP"){
                            options.isFASTTSP = true;
                        }
                        else{
                            options.isOPTTSP = true;
                        }
                        break;
                    }
                    case 'h':
                        cout << " You need to blast your way out and make the most of each dynamite stick by blasting the piles of rubble which take the fewest sticks to destroy\n";
                        exit(0);
                        break;

                    default:
                        cerr << "Unknown command line option" << endl;
                        exit(1);
                }  
            }   
        }

        void readInput(){
            size_t numV;
            cin >> numV;
            Vertex v;
            vertexCharact vChar;

            for(size_t i = 0; i < numV; i++){
                cin >> v.x >> v.y;
                if(v.x < 0 && v.y < 0){
                    v.campus = 'e'; // medical
                }
                else if((v.x == 0 && v.y < 0) || (v.y == 0 && v.x <0) || (v.x == 0 && v.y == 0)){
                    v.campus = 'b'; // border
                }
                else{
                    v.campus = 'a'; // main
                }
                map.push_back(v);
                mapVChar.push_back(vChar);
            }
        }

        void modeExecuter(){
            if(options.isMST == true){
                double w = MSTmode(map, mapVChar);   
                printMST(w);
            }
            else if(options.isFASTTSP == true){
                auto [tourLen, tour] = FASTTSPmode();
                printB(tourLen, tour);
            }
            else{
                OPTTSPmode();
            }
        }
};

int main(int argc, char *argv[]){
    ios_base::sync_with_stdio(false); // you should already have this
    cout << std::setprecision(2); //Always show 2 decimal places
    cout << std::fixed; //Disable scientific notation for large numbers

    // DEBUGGING
    // Do this once, in main()
    // cerr << fixed << showpoint << setprecision(2) << boolalpha;
    // END DEBUGGING

    drone idk;
    idk.getMode(argc, argv);
    idk.readInput();
    idk.modeExecuter();
    return 0;
}