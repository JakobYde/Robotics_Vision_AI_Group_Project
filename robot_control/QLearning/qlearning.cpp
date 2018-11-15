#include "qlearning.h"

QLearning::QLearning(std::string filename, std::string startState, float discount_rate, float stepSize, float greedy, float qInitValue, bool debug)
{
    std::cout << startState << std::endl;
    std::vector<state> statesTemp;
    srand (time(NULL));
    QLearning::discount_rate = discount_rate;
    QLearning::stepSize = stepSize;
    QLearning::greedy = greedy;
    QLearning::debug = debug;

    std::vector<std::vector<std::vector<std::string>>> stringvecsFromFile = stringFromFile(filename);

    if(stringvecsFromFile.size() > sizeof(unsigned long long int)*8){
        std::cout << "ERROR :::: To many states. Max number of stats is: " << sizeof(unsigned long long int)*8 << std::endl;
        std::exit(1);
    }

    for(int i = 0; i < stringvecsFromFile.size(); i++){
        states.push_back(std::vector<state>());
        for(int j = 0; j < std::pow(2,stringvecsFromFile.size()); j++){
            states.at(i).push_back(state());
        }
        stateNameIndex[stringvecsFromFile.at(i).at(STATE_NAME_INDEX).front()] = i;
        visest.push_back(false);
    }

    for(std::vector<std::vector<std::string>> stateLine : stringvecsFromFile){
        state newstate;

        std::string stateName = stateLine.at(STATE_NAME_INDEX).front();
        newstate.name = stateName;


        float x = std::stof(stateLine.at(STATE_X_INDEX).front());
        newstate.x = x;
        float y = std::stof(stateLine.at(STATE_Y_INDEX).front());
        newstate.y = y;

        float mean = std::stof(stateLine.at(STATE_MEAN_INDEX).front());
        float stddev = std::stof(stateLine.at(STATE_STDDIV_INDEX).front());
        newstate.mean = mean;
        newstate.stddev = stddev;

        statesTemp.push_back(newstate);
    }

    if(debug) std::cout << "Compling stats :::::" << std::endl;
    for(int i = 0; i < stringvecsFromFile.size(); i++){
        std::string stateName = stringvecsFromFile.at(i).at(STATE_NAME_INDEX).front();
        if(debug) std::cout << "\t state name: " << stateName << " and i is: " << i << " nameToIndexMap is: " << stateNameIndex[stateName] <<  std::endl;

        for(int j = 0; j < std::pow(2,stringvecsFromFile.size()); j++){
            states.at(i).at(j) = statesTemp.at(stateNameIndex[stateName]);

            for(std::string connName : stringvecsFromFile.at(i).at(STATE_CONN_INDEX)){
                int connBaseIndex = stateNameIndex[connName];
                state * connState = &states.at(connBaseIndex).at(j);
                states.at(i).at(j).actionStates.push_back(connState);
                states.at(i).at(j).qValues.push_back(qInitValue);
            }
        }
    }

    currentStateIndex = stateNameIndex[startState];
    std::cout << currentStateIndex << std::endl;
}

std::string QLearning::toBits(int n) {
    std::bitset<64> bit(n);
    std::string bits = bit.to_string();
    std::string bitstring = "[ ";
    for(int i = 0; i < states.size(); i++){
        bitstring += states.at(i).at(0).name + " = " + bits[64-i-1] + ", ";
    }
    return bitstring + "]";
}

void QLearning::print_stats(){
    std::cout << "[ ";
    for(int g = 0; g < visest.size()-1; g++){
        std::cout << states.at(g).at(0).name << " = " << visest.at(g) << ", ";
    }
    std::cout << states.back().at(0).name << " = " << visest.back() << " ]";
    for(int i = 0; i < states.size(); i++){
        std::cout << states.at(i).at(0).name << " | " << "(" << states.at(i).at(0).x << ", " << states.at(i).at(0).y << ") | " << "mean:" << states.at(i).at(0).mean << ", stdDiv:" << states.at(i).at(0).stddev << std::endl;
        for(int j = 0; j < states.at(i).size(); j++){
            state * st = & states.at(i).at(j);
            std::cout << "\t ";
            for(state* actPnt : st->actionStates) std::cout << actPnt->name << " ";
            std::cout << "| QValues: ";
            for(int k = 0; k < st->qValues.size(); k++){
                std::cout << st->actionStates.at(k)->name << " = " << st->qValues.at(k) << " ";
            }

            std::cout << toBits(j, states.size());
            std::cout << std::endl;
        }
    }
}

void QLearning::stripWhitespace(std::string& s){
    std::string tmp = "";
    for(int i = 0; i < s.length(); i++) if( not std::isblank(s[i] , std::locale()) )tmp+=s[i];
    s=tmp;
}

std::vector<std::vector<std::vector<std::string>>> QLearning::stringFromFile(std::string filename){
    std::string line;
    std::ifstream myfile (filename);
    std::vector<std::vector<std::vector<std::string>>> stringvecsFromFile;
    if (myfile.is_open())
    {
        while ( std::getline (myfile,line))
        {
            if(line == "" or line[0] == '#') continue;
            stripWhitespace(line);

            if(debug) std::cout << "Line: " << line << std::endl;
            std::vector<std::vector<std::string>> vecStringsInLine;
            std::vector<std::string> stringsInSubLine;
            std::string tmp = "";

            for(int i = 0; i < line.length(); i++){
                //std::cout << "Tmp: " << tmp << " Size of stringInLine: "<< stringInLine.size()<< " Size of stringFromFile: "<< stringFromFile.size() <<std::endl;
                if(line[i]==';'){
                    stringsInSubLine.push_back(tmp);
                    vecStringsInLine.push_back(stringsInSubLine);
                    stringsInSubLine.clear();
                    tmp = "";
                }
                else if(line[i]==',') {
                    stringsInSubLine.push_back(tmp);
                    tmp = "";
                }
                else tmp += line[i];
            }
            stringsInSubLine.push_back(tmp);
            vecStringsInLine.push_back(stringsInSubLine);
            stringvecsFromFile.push_back(vecStringsInLine);
        }
        myfile.close();
    }
    else std::cout << "Unable to open file\n";
    return stringvecsFromFile;
}


float QLearning::getMaxQ(state* newstate)  //Returns the higest value in the qValues vector for a given state.
{
    float max_q = newstate->qValues[0];
    for(int i= 1; i < newstate->qValues.size() ; i++)
    {
        if(newstate->qValues[i] > max_q)
        {
            max_q = newstate->qValues[i];
        }
    }
    return max_q;
}

float QLearning::runNormal_distribution(float neam, float stddev){
    std::normal_distribution<float> distribution(neam,stddev);
    return distribution(generator);
}

float QLearning::getReward(state* newstate) //should return reward of given state but return reward simulated by runNormal_distribution
{
    return runNormal_distribution(newstate->mean,newstate->stddev);
}

QLearning::state QLearning::getCurrentStarte(){
    if(debug) std::cout << "currentStateIndex: " << currentStateIndex << " calIndex(visest): " << calIndex(visest) << " State name is: " << states.at(currentStateIndex).at(calIndex(visest)).name << std::endl;
    return states.at(currentStateIndex).at(calIndex(visest));
}

int QLearning::getRandomactionIndex(){
    return rand()%states.at(currentStateIndex).at(calIndex(visest)).actionStates.size();
}

int QLearning::getMaxactionIndex(){
    int index = 0;
    float max_q = states.at(currentStateIndex).at(calIndex(visest)).qValues.at(0);
    for(int i = 1; i < states.at(currentStateIndex).at(calIndex(visest)).qValues.size(); i++){
        if(states.at(currentStateIndex).at(calIndex(visest)).qValues.at(i) > max_q){
            index = i;
            max_q = states.at(currentStateIndex).at(calIndex(visest)).qValues.at(i);
        }
    }
    return index;
}

// e-ereedy
int QLearning::policy(){
    float chance = (rand()%10000)/10000.0;
    int actionIndex;
    int actionIndexMax = getMaxactionIndex();
    if(greedy >= chance){
        if(debug) std::cout << "QDEBUG :::: Taking greedy action.";
        actionIndex = actionIndexMax;
    }
    else{
        if(debug) std::cout << "QDEBUG :::: Taking random action.";
        actionIndex = getRandomactionIndex();
        //while (currentState->actionStates.size() > 1 and actionIndex == actionIndexMax) {
           //actionIndex = getRandomactionIndex();
        //}
    }
    if(debug) std::cout << " Greedy is: " << greedy << " Chanse was: " << chance << ". From state: " << states.at(currentStateIndex).at(calIndex(visest)).name << " to state: " << states.at(currentStateIndex).at(calIndex(visest)).actionStates.at(actionIndex)->name << std::endl;
    return actionIndex;
}

//Choose A from S using policy derived from Q (e.g., e-ereedy
QLearning::state* QLearning::getNewState(){
    nextStateActionIndex = policy();
    return states.at(currentStateIndex).at(calIndex(visest)).actionStates.at(nextStateActionIndex);
}

//Take action A, observe R, S'
void QLearning::giveReward(float r){
    std::vector<bool> visests = visest;
    visests.at(currentStateIndex) = true;

    state* newState = states.at(currentStateIndex).at(calIndex(visests)).actionStates.at(nextStateActionIndex);

    float qNow = states.at(currentStateIndex).at(calIndex(visest)).qValues.at(nextStateActionIndex);
    float maxQ = getMaxQ(newState);
    states.at(currentStateIndex).at(calIndex(visest)).qValues.at(nextStateActionIndex) = qNow + stepSize*(r + discount_rate*maxQ - qNow);

    currentStateIndex = stateNameIndex[newState->name];
    visest = visests;
}

void QLearning::simulateActionReward(){
    state* action = getNewState();
    float reward = runNormal_distribution(action->mean, action->stddev);
    giveReward(reward);
}

void QLearning::wirteJSON(std::string filename){
    std::vector<Json> jsons;

    for(int i = 0; i < states.size(); i++){
        for(int j = 0; i < states.at(i).size(); j++){
            state * st = & states.at(i).at(j);
            Json jst;

            jst.add("name",st->name);
            jst.add("xy",std::vector<float>({st->x,st->y}));
            jst.add("mean",st->mean);
            jst.add("stddev",st->stddev);
            Json conn;
            for(int k = 0; k < st->actionStates.size(); k++){
                conn.add(st->actionStates.at(k)->name,st->qValues.at(k));
            }
            jst.add("conn", conn);
            jsons.push_back(jst);
        }
    }

    Json j;
    j.add("stats",jsons);
    j.write(filename);
}

unsigned long long int QLearning::calIndex(std::vector<bool> visests){
    unsigned long long int index = 0;
    for(int i = 0; i < visests.size(); i++) if(visests.at(i)) index += std::pow(2,i);
    return index;
}
