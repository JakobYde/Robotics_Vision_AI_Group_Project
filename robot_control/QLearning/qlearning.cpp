#include "qlearning.h"

QLearning::QLearning(std::string filename)
{
    std::vector<std::vector<std::vector<std::string>>> stringvecsFromFile = stringFromFile(filename);

    std::unordered_map<std::string, state*> pointMap;
    for(std::vector<std::vector<std::string>> stateLine : stringvecsFromFile){
        state newstate;
        states.push_back(newstate);
        state* statePnt = &states.back();

        std::string stateName = stateLine.at(STATE_NAME_INDEX).front();
        statePnt->name = stateName;
        pointMap[stateName] = statePnt;

        float x = std::stof(stateLine.at(STATE_X_INDEX).front());
        statePnt->x = x;
        float y = std::stof(stateLine.at(STATE_Y_INDEX).front());
        statePnt->y = y;

        float mean = std::stof(stateLine.at(STATE_MEAN_INDEX).front());
        statePnt->mean = mean;
        float stdDiv = std::stof(stateLine.at(STATE_STDDIV_INDEX).front());
        statePnt->stdDiv = stdDiv;
    }

    for(std::vector<std::vector<std::string>> stateLine : stringvecsFromFile){
        std::string stateName = stateLine.at(STATE_NAME_INDEX).front();
        state* statePnt = pointMap[stateName];

        std::vector<state*> connection;
        for(std::string connName : stateLine.at(3)){
            connection.push_back(pointMap[connName]);
        }
        statePnt->actionStates = connection;
    }

}

void QLearning::print_stats(){
    for(state st : states){
        std::cout << st.name << " | " << "(" << st.x << ", " << st.y << ") | " << "mean:" <<st.mean << ", stdDiv:" << st.stdDiv << " | Conection: ";
        for(state* actPnt : st.actionStates) std::cout << actPnt->name << " ";
        std::cout << "| QValues: ";
        for(float qval : st.qValues) std::cout << qval << " ";
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

/*
float QLearning::getMaxQ(state* newstate)  //Returns the higest value in the qValues vector for a given state.
{
    float max_q =newstate->qValues[0];
    for(int i= 1; i < newstate->qValues.size ; i++)
    {
        if(newstate->qValues[i]>max_q)
        {
            max_q = newstate->qValues[i];
        }
    }
    return max_q;
}


float QLearning::getReward(state* newstate) //should return reward of given state.
{
    normal_distribution<float> distribution(newstate->mean,newstate->stdDiv);

    return 0
}
*/
