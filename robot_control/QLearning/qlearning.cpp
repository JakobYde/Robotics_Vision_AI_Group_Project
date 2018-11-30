#include "qlearning.h"

QLearning::QLearning(std::string filename, std::string startState, float discount_rate, float stepSize, float greedy, float qInitValue, bool debug)
{
    std::vector<state> statesTemp;
    srand (time(NULL));
    generator.seed(time(NULL));
    QLearning::discount_rate = discount_rate;
    QLearning::stepSize = stepSize;
    QLearning::greedy = greedy;
    QLearning::debug = debug;
    QLearning::ininQValue = qInitValue;
    QLearning::startState = startState;

    std::vector<std::vector<std::vector<std::string>>> stringvecsFromFile = stringFromFile(filename);

    if(stringvecsFromFile.size() > sizeof(unsigned long long int)*8)
    {
        std::cout << "ERROR :::: To many states. Max number of stats is: " << sizeof(unsigned long long int)*8 << std::endl;
        std::exit(1);
    }

    //extracts states from the file stats.tex and puts them into a vector of vectors of states. Visits is initialized as false
    for(unsigned int i = 0; i < stringvecsFromFile.size(); i++)
    {
        states.push_back(std::vector<state>());
        for(int j = 0; j < std::pow(2,stringvecsFromFile.size()); j++)
        {
            states.at(i).push_back(state());
        }
        stateNameIndex[stringvecsFromFile.at(i).at(STATE_NAME_INDEX).front()] = i;
        visits.push_back(false);
    }

    for(std::vector<std::vector<std::string>> stateLine : stringvecsFromFile)
    {
        state newstate;

        std::string stateName = stateLine.at(STATE_NAME_INDEX).front();
        float x = std::stof(stateLine.at(STATE_X_INDEX).front());
        float y = std::stof(stateLine.at(STATE_Y_INDEX).front());
        float mean = std::stof(stateLine.at(STATE_MEAN_INDEX).front());
        float stddev = std::stof(stateLine.at(STATE_STDDIV_INDEX).front());
        newstate.name = stateName;
        newstate.x = x;
        newstate.y = y;
        newstate.mean = mean;
        newstate.stddev = stddev;

        statesTemp.push_back(newstate);  // all the base states are saved in statesTemp
    }

    if(debug) std::cout << "Compiling stats :::::" << std::endl;
    for(unsigned int i = 0; i < stringvecsFromFile.size(); i++){
        std::string stateName = stringvecsFromFile.at(i).at(STATE_NAME_INDEX).front();
        if(debug) std::cout << "\t state name: " << stateName << " and i is: " << i << " nameToIndexMap is: " << stateNameIndex[stateName] <<  std::endl;

        for(unsigned int j = 0; j < std::pow(2,stringvecsFromFile.size()); j++)//Husk der er en fejl her !!!!!!!
        {
            states.at(i).at(j) = statesTemp.at(stateNameIndex[stateName]); //Basestates are copied to all substates indexed by j

            for(std::string connName : stringvecsFromFile.at(i).at(STATE_CONN_INDEX))
            {
                unsigned int connBaseIndex = stateNameIndex[connName];
                unsigned long long int connectionJ = setBit(j,i,connBaseIndex);
                state * connState = &states.at(connBaseIndex).at(connectionJ); //look up functions to integer


                states.at(i).at(j).actionStates.push_back(connState);  //This is where we push back the different actions that each state has available.
                states.at(i).at(j).qValues.push_back(qInitValue);
            }
        }
    }
    currentStat = &states.at(stateNameIndex[startState]).at(0);
    //currentStateIndex = stateNameIndex[startState];
}

unsigned long long int QLearning::setBit(unsigned long long  int j, unsigned int i){
    std::bitset<sizeof(unsigned long long int)*8> bit(j);
    bit[i] = true;
    return bit.to_ullong();
}

unsigned long long int QLearning::setBit(unsigned long long  int j, unsigned int i, unsigned int g){
    std::bitset<sizeof(unsigned long long int)*8> bit(j);
    bit[i] = true;
    bit[g] = true;
    return bit.to_ullong();
}

std::string QLearning::toBits(unsigned int n) {
    std::bitset<sizeof(unsigned long long int)*8> bit(n);
    std::string bits = bit.to_string();
    std::string bitstring = "[ ";
    for(unsigned int i = 0; i < states.size(); i++){
        bitstring += states.at(i).at(0).name + " = " + bits[64-i-1] + ", ";
    }
    return bitstring + "]";
}

void QLearning::print_stats(){
    std::cout << "[ ";
    for(unsigned int g = 0; g < visits.size()-1; g++){
        std::cout << states.at(g).at(0).name << " = " << visits.at(g) << ", ";
    }
    std::cout << states.back().at(0).name << " = " << visits.back() << " ]";
    for(unsigned int i = 0; i < states.size(); i++){
        std::cout << states.at(i).at(0).name << " | " << "(" << states.at(i).at(0).x << ", " << states.at(i).at(0).y << ") | " << "mean:" << states.at(i).at(0).mean << ", stdDiv:" << states.at(i).at(0).stddev << std::endl;
        for(unsigned int j = 0; j < states.at(i).size(); j++){
            state * st = & states.at(i).at(j);
            std::cout << "\t ";
            for(state* actPnt : st->actionStates) std::cout << actPnt->name << " ";
            std::cout << "| QValues: ";
            for(unsigned int k = 0; k < st->qValues.size(); k++){
                std::cout << st->actionStates.at(k)->name << " = " << st->qValues.at(k) << " ";
            }

            std::cout << toBits(j);
            std::cout << std::endl;
        }
    }
}

void QLearning::setState(std::string state)
{
    startState = state;
    currentStat = &states.at(stateNameIndex[state]).at(0);
    //currentStateIndex = stateNameIndex[state];
    for(unsigned int i = 0; i < visits.size(); i++) visits.at(i) = false;
}
std::vector<std::string> QLearning::getStats(){
    std::vector<std::string> st;
    for(auto set : stateNameIndex) st.push_back(set.first);
    return st;
}

std::vector<bool> QLearning::getVisits(){
    return visits;
}


bool QLearning::allVisits(){
    for(bool vi : visits) if(not vi) return false;
    return true;
}

void QLearning::clear(std::string state)
{
    for(unsigned int i = 0; i < states.size(); i++){
        for(unsigned int j = 0; j < states.at(i).size(); j++){
            for(unsigned int k = 0; k < states.at(i).at(j).qValues.size(); k++) states.at(i).at(j).qValues.at(k) = ininQValue;
        }
    }
    setState(state);
    clearRewardHistroic();
}

void QLearning::clear(){
    QLearning::clear(startState);
}

float QLearning::getAvgReward(){
    float sum = 0.0;
    for(rewardH rh : rewardHistroic) sum += rh.r;
    return sum/float(rewardHistroic.size());
}
void QLearning::clearRewardHistroic(){
    rewardHistroic.clear();
}


void QLearning::stripWhitespace(std::string& s){
    std::string tmp = "";
    for(unsigned int i = 0; i < s.length(); i++) if( not std::isblank(s[i] , std::locale()) )tmp+=s[i];
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

            for(unsigned int i = 0; i < line.length(); i++){
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
    for(unsigned int i= 1; i < newstate->qValues.size() ; i++)
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
    int stateIndex = stateNameIndex[newstate->name];
    bool stateVisiset = visits.at(stateIndex);

    if(not stateVisiset) return runNormal_distribution(newstate->mean,newstate->stddev);
    return -5;
}

QLearning::state *QLearning::getCurrentStarte(){
    if(debug) std::cout << " State name is: " << currentStat->name << std::endl;
    return currentStat;
}

int QLearning::getRandomactionIndex(){
    return rand()%currentStat->actionStates.size();
}

std::vector<int> QLearning::getMaxactionIndexs(){
    std::vector<int> indexs;

    indexs.push_back(0);
    float max_q = currentStat->qValues.at(0);

    for(unsigned int i = 1; i < currentStat->qValues.size(); i++){
        if(currentStat->qValues.at(i) > max_q){
            max_q = currentStat->qValues.at(i);
            indexs.clear();
            indexs.push_back(i);
        }
        else if(currentStat->qValues.at(i) == max_q) indexs.push_back(i);
    }

    return indexs;
}

bool QLearning::inVec(std::vector<int> vec, int a){
    for(int temp : vec) if(a == temp) return true;
    else return false;
}

// e-ereedy
int QLearning::policy(){
    float chance = (rand()%10001)/10000.0;
    int actionIndex;
    std::vector<int> actionIndexMaxs = getMaxactionIndexs();
    if(1-greedy >= chance){
        if(debug) std::cout << "QDEBUG :::: Taking greedy action.";
        actionIndex = actionIndexMaxs.at(rand()%actionIndexMaxs.size());
    }
    else{
        if(debug) std::cout << "QDEBUG :::: Taking random action.";
        actionIndex = getRandomactionIndex();
        if(currentStat->actionStates.size() == 1 or currentStat->actionStates.size() == actionIndexMaxs.size()) return actionIndex;

        while (inVec(actionIndexMaxs,actionIndex)) {
           actionIndex = getRandomactionIndex();
        }
    }
    if(debug) std::cout << " Greedy is: " << greedy << " Chanse was: " << chance << ". From state: " << currentStat->name << " to state: " << currentStat->actionStates.at(actionIndex)->name << std::endl;
    return actionIndex;
}

//Choose A from S using policy derived from Q (e.g., e-ereedy
QLearning::state* QLearning::getNewState(){
    //visits.at(currentStateIndex) = true;

    nextStateActionIndex = policy();

    preStat = currentStat;

    currentStat = currentStat->actionStates.at(nextStateActionIndex);
    return currentStat;
}

//Take action A, observe R, S'. Use reward to update q-values for the corresponding action.
void QLearning::giveReward(float r){
    rewardHistroic.push_back(rewardH(r,currentStat->name));   //rewardHistroic contains a list of previously given rewards

    //state* newState = states.at(currentStateIndex).at(calIndex(visits)).actionStates.at(nextStateActionIndex);  // saves the pre-visits updated action_state for current state. the action-states are the states that can be moved to as a result of an action

    float qNow = preStat->qValues.at(nextStateActionIndex);
    //float qNow = states.at(currentStateIndex).at(calIndex(visits)).qValues.at(nextStateActionIndex);  // currently gets the qvalues of actionstate after updating currentstate, that is the

    float maxQ = getMaxQ(currentStat);
    //float maxQ = getMaxQ(newState);

    preStat->qValues.at(nextStateActionIndex) = qNow + stepSize*(r + discount_rate*maxQ - qNow);
    //states.at(currentStateIndex).at(calIndex(visits)).qValues.at(nextStateActionIndex) = qNow + stepSize*(r + discount_rate*maxQ - qNow);

    //visits.at(currentStateIndex) = true;
    visits.at(stateNameIndex[currentStat->name]) = true;
    //currentStateIndex = stateNameIndex[newState->name];
}

void QLearning::simulateActionReward(){
    state* action = getNewState();
    float reward = getReward(action);
    giveReward(reward);
}

void QLearning::wirteJSON(std::string filename){
    std::vector<Json> jsons;

    for(unsigned int i = 0; i < states.size(); i++){
        for(unsigned int j = 0; i < states.at(i).size(); j++){
            state * st = & states.at(i).at(j);
            Json jst;

            jst.add("name",st->name);
            jst.add("xy",std::vector<float>({st->x,st->y}));
            jst.add("mean",st->mean);
            jst.add("stddev",st->stddev);
            Json conn;
            for(unsigned int k = 0; k < st->actionStates.size(); k++){
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

//converts the visits bitset to an integer value that can be used to access states
unsigned long long int QLearning::calIndex(std::vector<bool> visits){
    unsigned long long int index = 0;
    for(unsigned int i = 0; i < visits.size(); i++) if(visits.at(i)) index += std::pow(2,i);
    return index;
}
