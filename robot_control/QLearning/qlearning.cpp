#include "qlearning.h"

QLearning::QLearning(std::string filename, std::string startState, float learningRate, float stepSize, float greedy, float qInitValue)
{
    srand (time(NULL));
    QLearning::learningRate = learningRate;
    QLearning::stepSize = stepSize;
    QLearning::greedy = greedy;

    std::vector<std::vector<std::vector<std::string>>> stringvecsFromFile = stringFromFile(filename);


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

        states.push_back(newstate);
        nameToIndexMap[stateName] = states.size()-1;
    }

    for(std::vector<std::vector<std::string>> stateLine : stringvecsFromFile){
        std::string stateName = stateLine.at(STATE_NAME_INDEX).front();
        int stateIndex = nameToIndexMap[stateName];

        std::vector<state*> connection;
        for(std::string connName : stateLine.at(STATE_CONN_INDEX)){
            connection.push_back(&states.at(nameToIndexMap[connName]));
            states.at(stateIndex).qValues.push_back(qInitValue);

        }
        states.at(stateIndex).actionStates = connection;
    }
    currentState = &states.at(nameToIndexMap[startState]);

}

void QLearning::print_stats(){
    for(state st : states){
        std::cout << st.name << " | " << "(" << st.x << ", " << st.y << ") | " << "mean:" <<st.mean << ", stdDiv:" << st.stddev << " | Conection: ";
        for(state* actPnt : st.actionStates) std::cout << actPnt->name << " ";
        std::cout << "| QValues: ";
        for(int i = 0; i < st.qValues.size(); i++){
            std::cout << st.actionStates.at(i)->name << " = " << st.qValues.at(i) << " ";
        }
        std::cout << std::endl;
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

            std::cout << "Line: " << line << std::endl;
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

int QLearning::getRandomactionIndex(){
    return rand()%currentState->actionStates.size();
}

int QLearning::getMaxactionIndex(){
    int index = 0;
    float max_q = currentState->qValues.at(0);
    for(int i = 1; i < currentState->qValues.size(); i++){
        if(currentState->qValues.at(i) > max_q){
            index = i;
            max_q = currentState->qValues.at(i);
        }
    }
    return index;
}

// e-ereedy
int QLearning::policy(){
    float chance = (rand()%10000)/10000.0;
    int actionIndex;
    if((1-greedy) <= chance) actionIndex = getMaxactionIndex();
    else actionIndex = getRandomactionIndex();

    return actionIndex;
}

//Choose A from S using policy derived from Q (e.g., e-ereedy
QLearning::state* QLearning::getNewState(){
    nextStateActionIndex = policy();
    return currentState->actionStates.at(nextStateActionIndex);
}

//Take action A, observe R, S'
void QLearning::giveReward(float r){
    state* newState = currentState->actionStates.at(nextStateActionIndex);

    float qNow = currentState->qValues.at(nextStateActionIndex);
    float maxQ = getMaxQ(newState);
    currentState->qValues.at(nextStateActionIndex) = qNow + stepSize*(r + learningRate*maxQ-qNow);

    currentState = newState;
}

void QLearning::simulateActionReward(){
    state* action = getNewState();
    float reward = runNormal_distribution(action->mean, action->stddev);
    giveReward(reward);
}

void QLearning::wirteJSON(std::string filename){
    std::vector<Json> jsons;
    for(state st : states){
        Json jst;

        jst.add("name",st.name);
        jst.add("xy",std::vector<float>({st.x,st.y}));

        Json conn;
        for(int i = 0; i < st.actionStates.size(); i++){
            conn.add(st.actionStates.at(i)->name,st.qValues.at(i));
        }
        jst.add("conn", conn);
        jsons.push_back(jst);
    }
    Json j;
    j.add("stats",jsons);
    j.write(filename);
}
