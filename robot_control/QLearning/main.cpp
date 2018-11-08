#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include "qlearning.h"

int main()
{
    enum part { p_name, p_x, p_y, p_conn };
    struct state
    {
        state() {}

        std::vector<state*> actionStates;
        std::vector<float> qValues;

        float x, y;

        float stdDiv;
        float mean;
    };
    struct stateAndConn
    {
        stateAndConn() {}
        stateAndConn(state stIn, std::string connIn) {
            st = stIn;
            conn = connIn;
        }
        state st;
        std::string conn;
    };
    std::unordered_map<std::string, stateAndConn> statesMap;
    std::vector<state> states;

    std::string line;
    std::ifstream myfile ("stats.txt");
    if (myfile.is_open())
    {
        while ( std::getline (myfile,line) )
        {
            state st;
            std::string name;
            part correntPart = p_name;
            std::string tmp = "";
            std::cout << "The line is: " << line << '\n';
            for(int i = 0; i < line.length(); i++){
                switch (correntPart) {
                    case p_name:
                        if(line[i]==';'){
                            correntPart = p_x;
                            tmp = "";
                        }
                        else{
                            name += line[i];
                        }

                        break;
                    case p_x:
                        if(line[i]==';'){
                            correntPart = p_y;
                            st.x = std::stof(tmp);
                            tmp = "";
                        }
                        else{
                            tmp += line[i];
                        }
                        break;
                    case p_y:
                        if(line[i]==';'){
                            correntPart = p_conn;

                            st.y = std::stof(tmp);
                            tmp = "";
                        }
                        else{
                            tmp += line[i];
                        }
                        break;
                    case p_conn:
                        tmp += line[i];
                        break;
                    default:
                        break;
                }
                std::cout << "I char is: " << line[i] << ", tmp is: " << tmp << std::endl;
            }
            statesMap[name] = stateAndConn(st,tmp);
        }
        myfile.close();

        for(auto& pair : statesMap){
            std::cout << pair.first << " - " << pair.second.conn << std::endl;

        }

    }
    else std::cout << "Unable to open file\n";



    return 0;
}
