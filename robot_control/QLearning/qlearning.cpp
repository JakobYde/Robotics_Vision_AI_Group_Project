#include "qlearning.h"

QLearning::QLearning(std::string filename, std::string startState, float discount_rate, float stepSize, float greedy, float qInitValue, bool useMultiQ, unsigned int numberOfQValues, bool debug)
{
    //Set lerning parameters
    QLearning::useMultiQ = useMultiQ;
    std::vector<state> statesTemp;
    srand (time(NULL));
    generator.seed(time(NULL));
    QLearning::discount_rate = discount_rate;
    QLearning::stepSize = stepSize;
    QLearning::greedy = greedy;
    QLearning::debug = debug;
    QLearning::ininQValue = qInitValue;
    QLearning::startState = startState;
    QLearning::numberOfQValues = numberOfQValues;

    std::vector<std::vector<std::vector<std::string>>> stringvecsFromFile = stringFromFile(filename);//Reads the file with stats

    if(stringvecsFromFile.size() > sizeof(unsigned long long int)*8)//If there is more stats (sub stats) than a unsigned long long int can index the program is stop
    {
        std::cout << "ERROR :::: To many states. Max number of stats is: " << sizeof(unsigned long long int)*8 << std::endl;
        std::exit(1);
    }

    //extracts states from the file stats.tex and puts them into a vector of vectors of states. Visits is initialized as false
    for(unsigned int i = 0; i < stringvecsFromFile.size(); i++)
    {
        std::vector<std::vector<std::string>> stateLine = stringvecsFromFile.at(i);//Get the base state parameter vector
        std::string stateName = stateLine.at(STATE_NAME_INDEX).front(); //Gets the base state name

        //Allocate memory for alle sub states to the base state
        states.push_back(std::vector<state>());
        for(int j = 0; j < std::pow(2,stringvecsFromFile.size()); j++)
        {
            states.at(i).push_back(state());
        }

        stateNameIndex[stateName] = i; //Save at with index the current base state is saved
        visits.push_back(false);//Makes a bool in the visits vector for the base state

        //Make base state
        state newstate;

        float x = std::stof(stateLine.at(STATE_X_INDEX).front());
        float y = std::stof(stateLine.at(STATE_Y_INDEX).front());
        float mean = std::stof(stateLine.at(STATE_MEAN_INDEX).front());
        float stddev = std::stof(stateLine.at(STATE_STDDIV_INDEX).front());
        newstate.name = stateName;
        newstate.x = x;
        newstate.y = y;
        newstate.mean = mean;
        newstate.stddev = stddev;

        statesTemp.push_back(newstate);  // all the base states are saved in statesTemp so sub stats can be made from a copi of them

    }

    if(debug) std::cout << "Compiling stats :::::" << std::endl;
    for(unsigned int i = 0; i < stringvecsFromFile.size(); i++){ //runs over all base stats
        std::string stateName = stringvecsFromFile.at(i).at(STATE_NAME_INDEX).front(); //Gets the name fro the base state
        if(debug) std::cout << "\t state name: " << stateName << " and i is: " << i << " nameToIndexMap is: " << stateNameIndex[stateName] <<  std::endl;

        for(unsigned int j = 0; j < std::pow(2,stringvecsFromFile.size()); j++)//Run over all sub states
        {
            states.at(i).at(j) = statesTemp.at(stateNameIndex[stateName]); //Basestates are copied to all substates indexed by j

            for(std::string connName : stringvecsFromFile.at(i).at(STATE_CONN_INDEX))//Runs for ervy connection (action) the current state have
            {
                unsigned int connBaseIndex = stateNameIndex[connName]; //Get the base index of the connection state (the state the action is to) (So the index for the group of states of fx "S0")
                unsigned long long int connectionJ = setBit(j,i,connBaseIndex); //Get the index for the sub state witch the action has to point to
                state * connState = &states.at(connBaseIndex).at(connectionJ); //Finds the state from the two indexs


                states.at(i).at(j).actionStates.push_back(connState);  //This is where we push back the different actions that each state has available.
                states.at(i).at(j).qValues.push_back(std::vector<float>());//Make a empty vector for the q values for the action
                for(unsigned int k = 0; k < numberOfQValues; k++) states.at(i).at(j).qValues.back().push_back(qInitValue); //Push numberOfQValues qValues for the action
            }
        }
    }
    currentStat = &states.at(stateNameIndex[startState]).at(0); //Sets the start state
    //currentStateIndex = stateNameIndex[startState];
}

//Return j where the bit i is set
unsigned long long int QLearning::setBit(unsigned long long  int j, unsigned int i){
    std::bitset<sizeof(unsigned long long int)*8> bit(j);
    bit[i] = true;
    return bit.to_ullong();
}
//Return j where the bit i and g is set
unsigned long long int QLearning::setBit(unsigned long long  int j, unsigned int i, unsigned int g){
    std::bitset<sizeof(unsigned long long int)*8> bit(j);
    bit[i] = true;
    bit[g] = true;
    return bit.to_ullong();
}
//Return a string with the binari for n (so n=2 => "[ 0 0 0 ..(number of stats).. 1 0 ]"
std::string QLearning::toBits(unsigned int n) {
    std::bitset<sizeof(unsigned long long int)*8> bit(n);
    std::string bits = bit.to_string();
    std::string bitstring = "[ ";
    for(unsigned int i = 0; i < states.size(); i++){
        bitstring += states.at(i).at(0).name + " = " + bits[64-i-1] + ", ";
    }
    return bitstring + "]";
}
//Print infomation about the stats
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
                std::cout << st->actionStates.at(k)->name << " = [";
                for(unsigned int g = 0; g < st->qValues.at(k).size(); g++) std::cout << st->qValues.at(k).at(g) << " ";
                 std::cout << "] ";
            }

            std::cout << toBits(j);
            std::cout << std::endl;
        }
    }
}
//Set the current state to the sub state of the base state with the name given in state where none states are visists
void QLearning::setState(std::string state)
{
    startState = state;
    currentStat = &states.at(stateNameIndex[state]).at(0);
    //currentStateIndex = stateNameIndex[state];
    for(unsigned int i = 0; i < visits.size(); i++) visits.at(i) = false;
}
//Return a vector with the base state names
std::vector<std::string> QLearning::getStats(){
    std::vector<std::string> st;
    for(auto set : stateNameIndex) st.push_back(set.first);
    return st;
}
//Retrun with base states there have been visiits
std::vector<bool> QLearning::getVisits(){
    return visits;
}
//Retrun rather all stats have been visits
bool QLearning::allVisits(){
    for(bool vi : visits) if(not vi) return false;
    return true;
}
//Clear all and set the current state to the sub state of the base state with the name given in state
void QLearning::clear(std::string state)
{
    //Cleat all qvalues
    for(unsigned int i = 0; i < states.size(); i++){
        for(unsigned int j = 0; j < states.at(i).size(); j++){
            for(unsigned int k = 0; k < states.at(i).at(j).qValues.size(); k++){
                states.at(i).at(j).qValues.at(k).clear();
                for(unsigned int g = 0; g < numberOfQValues; g++) states.at(i).at(j).qValues.at(k).push_back(ininQValue); //Push numberOfQValues qValues for the action
            }
        }
    }
    //Set the current state to the sub state of the base state with the name given in state
    setState(state);
    //Clear the reward histroi
    clearRewardHistroic();
}
//Clear all and set the state to the same start state as given when QLerarning objekt was creaet
void QLearning::clear(){
    QLearning::clear(startState);
}
//Calulete the avg of the rewards saved in rewardHistroic (remember to clear rewardHistroic aftherwods)
float QLearning::getAvgReward(){
    float sum = 0.0;
    for(rewardH rh : rewardHistroic) sum += rh.r;
    return sum/float(rewardHistroic.size());
}
//Clers the rewardHistroic
void QLearning::clearRewardHistroic(){
    rewardHistroic.clear();
}
//Strips all white space charetes in given string
void QLearning::stripWhitespace(std::string& s){
    std::string tmp = "";
    for(unsigned int i = 0; i < s.length(); i++) if( not std::isblank(s[i] , std::locale()) )tmp+=s[i];
    s=tmp;
}
//Estrags all parametes from a given file at parth given in filename
std::vector<std::vector<std::vector<std::string>>> QLearning::stringFromFile(std::string filename){
    std::string line;
    std::ifstream myfile (filename); //Trys to open file
    std::vector<std::vector<std::vector<std::string>>> stringvecsFromFile;
    if (myfile.is_open()) //If the file is opend
    {
        while ( std::getline (myfile,line)) //Run over all lines
        {
            if(line == "" or line[0] == '#') continue; //If the line is emtpy or the line starts with a '#' the line is ignored
            stripWhitespace(line);//Strip all withspace from the line

            if(debug) std::cout << "Line: " << line << std::endl;
            std::vector<std::vector<std::string>> vecStringsInLine;//Vector for alle parts of the line
            std::vector<std::string> stringsInSubLine; //Vector for subparts of the line
            std::string tmp = "";

            for(unsigned int i = 0; i < line.length(); i++){ //Run over all chrters in the line
                //std::cout << "Tmp: " << tmp << " Size of stringInLine: "<< stringInLine.size()<< " Size of stringFromFile: "<< stringFromFile.size() <<std::endl;
                if(line[i]==';'){//If there is a ';' chareter there is a new part of the line
                    stringsInSubLine.push_back(tmp);
                    vecStringsInLine.push_back(stringsInSubLine);
                    stringsInSubLine.clear();
                    tmp = "";
                }
                else if(line[i]==',') {//If there is a ',' there is a new sub part of a part of the line
                    stringsInSubLine.push_back(tmp);
                    tmp = "";
                }
                else tmp += line[i];//Else the chareter most be a part of the last "part"
            }
            //The current line is done and push in to stringvecsFromFile
            stringsInSubLine.push_back(tmp);
            vecStringsInLine.push_back(stringsInSubLine);
            stringvecsFromFile.push_back(vecStringsInLine);
        }
        myfile.close();
    }
    else std::cout << "Unable to open file\n";
    return stringvecsFromFile;
}

//Returns the higest value in the qValues for the nQ'th qvalues for a given state.
float QLearning::getMaxQ(state* newstate, unsigned int nQ)
{
    //As a start geas the first values is set to be the max
    float max_q = newstate->qValues.at(0)[nQ];
    for(unsigned int i= 1; i < newstate->qValues.size() ; i++)
    {
        if(newstate->qValues.at(i)[nQ] > max_q)
        {
            max_q = newstate->qValues.at(i)[nQ];
        }
    }
    return max_q;
}
//Return a value from a notmal distribution with the given neam and stddev
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

std::vector<int> QLearning::getMaxActionIndexs(){
    std::vector<int> indexs;
    indexs.push_back(0);


    float max_q = currentStat->qValues.at(0)[0];
    if(numberOfQValues > 1 and useMultiQ) for(unsigned int i = 1; i < numberOfQValues; i++) max_q += currentStat->qValues.at(0)[i];


    for(unsigned int i = 1; i < currentStat->qValues.size(); i++){
        float currentQ = currentStat->qValues.at(i)[0];
        if(numberOfQValues > 1 and useMultiQ) for(unsigned int j = 1; j < numberOfQValues; j++) currentQ += currentStat->qValues.at(i)[j];

        if(currentQ > max_q){
            max_q = currentQ;
            indexs.clear();
            indexs.push_back(i);
        }
        else if(currentQ == max_q) indexs.push_back(i);
    }

    return indexs;
}

std::vector<int> QLearning::getMaxActionIndexs(unsigned int nQ){
    std::vector<int> indexs;
    indexs.push_back(0);

    float max_q = currentStat->qValues.at(0)[nQ];

    for(unsigned int i = 1; i < currentStat->qValues.size(); i++){
        float currentQ = currentStat->qValues.at(i)[nQ];

        if(currentQ > max_q){
            max_q = currentQ;
            indexs.clear();
            indexs.push_back(i);
        }
        else if(currentQ == max_q) indexs.push_back(i);
    }

    return indexs;
}

bool QLearning::inVec(std::vector<int> vec, int a){
    for(int temp : vec) if(a == temp) return true;
    return false;
}

// e-ereedy
int QLearning::policy(){
    float chance = (rand()%RAND_MAX)/float(RAND_MAX);
    int actionIndex;
    std::vector<int> actionIndexMaxs = getMaxActionIndexs();
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
    //In this funktion preStat is S
    //And currentStat is S'

    rewardHistroic.push_back(rewardH(r,currentStat->name));   //rewardHistroic contains a list of previously given rewards

    //Find q value for update and q value for action
    int qToUpdate = 0;
    int qForAction = 0;

    if(numberOfQValues > 1 and useMultiQ){
        qToUpdate = rand()%numberOfQValues;
        qForAction = rand()%numberOfQValues;

        while (qForAction == qToUpdate) qForAction = rand()%numberOfQValues;
    }

    float qNow = preStat->qValues.at(nextStateActionIndex)[qToUpdate];

    float maxQ = getMaxQ(currentStat, qToUpdate);
    if(numberOfQValues > 1 and useMultiQ){
        std::vector<int> actions = getMaxActionIndexs(qToUpdate);
        float action = actions.at(rand()%actions.size());

        maxQ = currentStat->qValues.at(action)[qForAction];

    }

    preStat->qValues.at(nextStateActionIndex)[qToUpdate] = qNow + stepSize*(r + discount_rate*maxQ - qNow);

    visits.at(stateNameIndex[currentStat->name]) = true;
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
