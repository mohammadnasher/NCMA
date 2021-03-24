//  main.cpp
//  NetworkSimulation
//
//  Created by Mohammad Nasher on 4/10/20.
//  Copyright © 2020 Mohammad Nasher. All rights reserved.
//

#include <iostream>
#include <vector>
#include <random>
#include <time.h>
#include <ctime>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>


#define __DEBUG_COMMAND__ true;

//Default required data to send for node A,B,C,D
//You can Change it to Simulatre with different data
auto constexpr __DEF_A_NODE_DATA__ = "10001011110000110111110110000111110001100011101010000111111011011101011010111010101010101011111111011101111010";
auto constexpr __DEF_B_NODE_DATA__ = "11101101110110101111110000111111110011010111111011101111110111 01111011110111 ";
auto constexpr __DEF_C_NODE_DATA__ = "10001011110000110111110110000111110001100011101010000111111011 01110101101011000011001110101000";
auto constexpr __DEF_D_NODE_DATA__ = "000100100001111110111010100001001011110101000011000110100100";
auto constexpr __DEF_CHANNEL_SIZE__ = 40;
auto constexpr __DEF_PROBABILITY_TO_SEND__ = 50;
auto constexpr __DEF_FILE_ADDRESS_4__ = "Matrix4.txt";
auto constexpr __DEF_FILE_ADDRESS_512__ = "Matrix512.txt";
auto constexpr __CDMA_TYPE__ = 512;

enum __Public_NodeID{A = 1, B , C , D};

enum __Protocol_Types{CDMA, TDMA, DYNAMIC_TDMA};

void _LogError(std::string _InpErr){
    std::cout << "[E]: <" << _InpErr << ">" <<std::endl;
}

void _LogDetail(std::string _InpDetail){
    std::cout << "[D]: " << _InpDetail << std::endl;
}

class Node{
private:
    int _ID;
    std::vector<bool> _Data;
    int _SentCheckPoint;
    int _DataRate;
    
public:
    Node(int _Inp_ID, std::vector<bool> _Inp_Data, int _Inp_DataRate){
        if(_Inp_Data.size() == 0 || _Inp_ID <= 0 || _Inp_DataRate <= 0){
            _LogError("INVALID INPUT IN NODE::NODE(INT, STD::VECTOR<BOOL>)");
            return;
        }
        this->_Data = _Inp_Data;
        this->_ID = _Inp_ID;
        this->_DataRate = _Inp_DataRate;
        this->_SentCheckPoint = 0;
    }
    Node(int _Inp_ID, std::string _Inp_Data, int _Inp_DataRate){
        if(_Inp_Data == "" || _Inp_ID <= 0 || _Inp_DataRate <= 0){
               _LogError("INVALID INPUT IN NODE::NODE(INT, STD::STRING)");
               return;
        }
        for(char tmp : _Inp_Data)
             this->_Data.push_back(((int)(tmp) - 48 == 1));
        this->_ID = _Inp_ID;
        this->_DataRate = _Inp_DataRate;
        this->_SentCheckPoint = 0;
    }
    Node(int _Inp_ID, int _Inp_DataSize, int _Inp_DataRate){
        if(_Inp_DataSize <= 0 || _Inp_ID <= 0 || _Inp_DataRate <= 0){
                   _LogError("INVALID INPUT IN NODE::NODE(INT, STD::STRING)");
                   return;
        }
        auto seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        std::uniform_int_distribution<int> distributionInteger(1, 10);
        for(auto i = 0 ; i < _Inp_DataSize; i++)
            this->_Data.push_back((distributionInteger(generator) % 2 == 1));
        this->_ID = _Inp_ID;
        this->_DataRate = _Inp_DataRate;
        this->_SentCheckPoint = 0;
    }
    bool SetData(std::string _Inp_Data){
        if(_Inp_Data == ""){
             _LogError("INVALID INPUT IN NODE::SETDATA(STD::STRING)");
            return false;
        }
        for(char tmp : _Inp_Data)
            this->_Data.push_back(((int)(tmp) - 48 == 1));
        return true;
    }
    bool SetID(int _Inp_ID){
        if(_Inp_ID <= 0){
            _LogError("INVALID INPUT IN NODE::SETID(INT)");
            return false;
        }
        this->_ID = _Inp_ID;
        return true;
    }
    auto GetSizeOfData(){
        return (int)this->_Data.size();
    }
    int GetID(){
        return this->_ID;
    }
    auto GetDataRate(){
        return this->_DataRate;
    }
    bool SetDataRate(int _Inp_DataRate){
        if(_Inp_DataRate <= 0){
            _LogError("INVALID INPUT IN NODE::SETDATARATE(INT)");
            return false;
        }
        this->_DataRate = _Inp_DataRate;
        return true;
    }
    std::vector<bool> SendData(){
        std::vector<bool> _res;
        //Available Data is less that data rate -> send all of remaining Data
        if(this->_Data.size() - this->_SentCheckPoint - 1 <= this->_DataRate){
            for(auto it = this->_SentCheckPoint; it < this->_Data.size() ; it++)
                _res.push_back(this->_Data[it]);
            //Increment CheckPoint
            this->_SentCheckPoint = (int)this->_Data.size() - 1;
        }
        //if there is more than data rate available data
        else{
            for(auto it = this->_SentCheckPoint; it < this->_SentCheckPoint + this->_DataRate; it++)
                _res.push_back(this->_Data[it]);
            this->_SentCheckPoint += this-> _DataRate;
        }
        //_LogError(std::to_string(_res.size()));
        return _res;
    }
    
    int Get1BitData(){
        return this->_Data[this->_SentCheckPoint++];
    }
    
    bool _IsReadyToSend(int _InpProbability){
           if(_InpProbability < 0 || _InpProbability > 100){
               _LogError("INVALID PROBABILITY TO SEND DATA HAS BEEN SET! CHECK IT OUT ...");
               return true;
           }
           if(this->_SentCheckPoint == this->_Data.size() - 1)
               return false;
           std::random_device device;
           std::mt19937 generator(device());
           std::uniform_real_distribution<double> distribution(1,100);
           return (distribution(generator) > _InpProbability) ?  false : true;
       }
    
    bool _IsDataDone(){return (this->_SentCheckPoint == this->_Data.size() - 1);}

};

class NetworkSimulatorEngin{
private:
    std::vector<Node> _NetworkNodes;
    std::vector<bool> _SharedChannel;
    int _TimeSlot;
    bool _IsChannelEmpty;
    int** _WalshCode;
    
    bool _IsDone(){
        for(Node node : this->_NetworkNodes)
            if(!node._IsDataDone())
                return false;
        return true;
    }
    
    std::vector<Node>::iterator _whoIs‌ThePriority(){
        int _SumDatatoSend = 0;
        for(Node N : this->_NetworkNodes)
            _SumDatatoSend += N.GetSizeOfData();
        auto seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        std::uniform_int_distribution<int> distributionInteger(1, _SumDatatoSend);
        int _Rand = distributionInteger(generator);
        int _CheckSumation = 1;
        for(auto index = this->_NetworkNodes.begin() ; index < this->_NetworkNodes.end(); index++){
            if(_CheckSumation + index->GetSizeOfData() >= _Rand){
                return index;
            }
            _CheckSumation += index->GetSizeOfData();
        }
        return this->_NetworkNodes.end();
    }
    
    
    int _HowManyTimeSlotToSend(std::vector<Node>::iterator _Inp_index){
        int _Res = 0;
        for(Node x : this->_NetworkNodes){
            if(x.GetSizeOfData() != 0)
                _Res += (x.GetSizeOfData() / x.GetDataRate());
        }
        _Res = _Res / this->_NetworkNodes.size();
        if(_Res < (_Inp_index->GetSizeOfData() / _Inp_index->GetDataRate()))
            return _Res;
        return (_Inp_index->GetSizeOfData() % _Inp_index->GetDataRate() == 0)? _Inp_index->GetSizeOfData() / _Inp_index->GetDataRate() : (_Inp_index->GetSizeOfData() / _Inp_index->GetDataRate()) + 1;
    }
    
    void LoadWalshMatrix(int _InpType){
        
        std::fstream _MatrixFile;
        switch (_InpType) {
            case 4:
                _MatrixFile.open(__DEF_FILE_ADDRESS_4__);
                if(!_MatrixFile){
                    _LogError("INVALID FILE ADDRESS IN <NETWORKSIMULATORENGIN::LOADWALSHMATRIX(INT)>");
                }else{
                    this->_WalshCode = new int*[4];
                    for(auto i = 0 ; i < 4 ; i++){
                        this->_WalshCode[i] = new int[4];
                    }
                    int _Counter_i = 0;
                    int _Counter_j = 0;
                    while(!_MatrixFile.eof() || _Counter_i == 4){
                        _MatrixFile >> this->_WalshCode[_Counter_i][_Counter_j];
                        _Counter_j++;
                        if(_Counter_j % 4 == 0){
                            _Counter_j = 0;
                            _Counter_i++;
                        }
                    }
                }
                break;
            case 512:
                _MatrixFile.open(__DEF_FILE_ADDRESS_512__);
                       if(!_MatrixFile){
                           _LogError("INVALID FILE ADDRESS IN <NETWORKSIMULATORENGIN::LOADWALSHMATRIX(INT)>");
                       }else{
                           this->_WalshCode = new int*[512];
                           for(auto i = 0 ; i < 512 ; i++){
                               this->_WalshCode[i] = new int[512];
                           }
                           int _Counter_i = 0;
                           int _Counter_j = 0;
                           while(!_MatrixFile.eof() || _Counter_i == 4){
                               _MatrixFile >> this->_WalshCode[_Counter_i][_Counter_j];
                               _Counter_j++;
                               if(_Counter_j % 512 == 0){
                                   _Counter_j = 0;
                                   _Counter_i++;
                               }
                           }
                       }
                break;
            default:
                _LogError("INVALID INPUT IN <NETWORKSIMULATORENGIN::LOADWALSHMATRIX(INT)>");
                break;
        }
    }
    
public:
    NetworkSimulatorEngin(std::vector<Node> _Inp_Nodes){
        if(_Inp_Nodes.size() == 0){
            _LogError("INVALID INPUT IN NETWORKSIMULATIONENGIN::NETWORKSIMULATIONENGINE(STD::VECTOR<NODE>, INT)");
            return;
        }
        this->_NetworkNodes = _Inp_Nodes;
        for (auto i = 0; i < __DEF_CHANNEL_SIZE__; i++)
            this->_SharedChannel.push_back(0);
        this->_IsChannelEmpty = true;
        this->_TimeSlot = 0;
    }
    
    void StartSimulation(int _ProtocolType){
       if(this->_NetworkNodes.size() == 0 || this->_SharedChannel.size() == 0)
       {
           _LogError("NETWORK PROPERTIES NOT SET <NETWROKSIMULATORENGINE::STARTSIMULATION(INT)>");
           return;
       }
        int _NumberOfRounds = 0;
        switch (_ProtocolType)
        {
            case CDMA:
                //Get Walsh Code From File
                this->LoadWalshMatrix(__CDMA_TYPE__);
                //While there is data to send
                while (!this->_IsDone()) {
                    _NumberOfRounds++;
                    int _RemainingChannelVol = __DEF_CHANNEL_SIZE__;
                    //for each node
                    for(int i = 0 ; i < this->_NetworkNodes.size(); i++){
                        //Ask is ready to send
                        if(_RemainingChannelVol > 0){
                            if(this->_NetworkNodes[i]._IsReadyToSend(__DEF_PROBABILITY_TO_SEND__)){
                                auto _DataBit = this->_NetworkNodes[i].Get1BitData();
                                _RemainingChannelVol--;
                                int* _Code = _WalshCode[i];
                                if(_DataBit == 0){
                                    for(int j = 0 ; j < __CDMA_TYPE__; j++)
                                        _Code[j] = -1 * _Code[j];
                                    #ifdef __DEBUG_COMMAND__
                                        _LogDetail("ID = " + std::to_string(i) + " SENT DATA. RESULT OF WALSH = -1.");
                                    #endif
                                }else{
                                    #ifdef __DEBUG_COMMAND__
                                        _LogDetail("ID = " + std::to_string(i) + " SENT DATA. RESULT OF WALSH = +1.");
                                    #endif
                                }
                            }
                        }
                        else{
                            #ifdef __DEBUG_COMMAND__
                            _LogDetail("<FULL> CHANNEL CAPACITY IS FULL. TRYING TO MAKE IT EMPTY ...");
                            #endif
                            break;
                        }
                    }
                }
                //#ifdef __DEBUG_COMMAND__
                _LogDetail("<DONE> ALL NODES HAVE SENT ALL DATA IN TOTAL " + std::to_string(_NumberOfRounds) + " ROUNDS");
                //#endif
                break;
            case TDMA:
                //Do Transmition Progress According TDMA Startegy
                this->_TimeSlot = 0;
                //While there is data to send
                while(!this->_IsDone()){
                    auto _indeX = this->_whoIs‌ThePriority();
                    //If have data to send and ready to send
                    if(_indeX->_IsReadyToSend(__DEF_PROBABILITY_TO_SEND__)){
                        auto _Sent = _indeX->SendData();
                        this->_TimeSlot++;
                        #ifdef __DEBUG_COMMAND__
                        _LogDetail("ID = " + std::to_string(_indeX->GetID()) + " SENT DATA IN  #TIMESLOT = " + std::to_string(this->_TimeSlot) + " DATA SIZE = " + std::to_string(_Sent.size()));
                        #endif
                    }
                    else{
                        #ifdef __DEBUG_COMMAND__
                        _LogDetail("<TIMESLOT_LOST> ID = " + std::to_string(_indeX->GetID()) + " IS NOT READY TO SEND DATA IN #TIMESLOT = " + std::to_string(_TimeSlot));
                        #endif
                        this->_TimeSlot++;
                    }
                }
                #ifdef __DEBUG_COMMAND__
                _LogDetail("<DONE> ALL NODES HAVE SENT ALL DATA IN TOTAL " + std::to_string(this->_TimeSlot) + " TIME SLOTS");
                #endif
                break;
            case DYNAMIC_TDMA:
                //Do Transmition Progress According Dynamic TDMA Startegy
                this->_TimeSlot = 0;
                //While there is Data to send
                while(!this->_IsDone()){
                    auto _indeX = this->_whoIs‌ThePriority();
                    //if have data to send ready to send
                    if(_indeX ->_IsReadyToSend(__DEF_PROBABILITY_TO_SEND__)){
                        int _TimesToSend = this->_HowManyTimeSlotToSend(_indeX);
                        for(auto i = 0 ; i < _TimesToSend; i++){
                            //If is ready to send
                            if(_indeX->_IsReadyToSend(__DEF_PROBABILITY_TO_SEND__)){
                                auto _Sent = _indeX -> SendData();
                                #ifdef __DEBUG_COMMAND__
                                _LogDetail("ID = " + std::to_string(_indeX->GetID()) + " SENT DATA IN  #TIMESLOT IN INTERVAL = " + std::to_string(i) + " DATA SIZE = " + std::to_string(_Sent.size()) + " #TOTAL TIMESLOT = " + std::to_string(this->_TimeSlot));
                                #endif
                                this->_TimeSlot++;
                            }
                            //if is not ready to send
                            else{
                                #ifdef __DEBUG_COMMAND__
                                _LogDetail("<TIMESLOT_LOST> ID = " + std::to_string(_indeX->GetID()) + " IS NOT READY TO SEND DATA IN TIMESLOTSW INTERVAL. #TIMESLOT IN INTERVAL = " + std::to_string(i) + " #TOTAL TIMESLOT = " + std::to_string(this->_TimeSlot));
                                #endif
                                this->_TimeSlot++;
                                break;
                            }
                        }
                    }
                    else{
                        #ifdef __DEBUG_COMMAND__
                        _LogDetail("<TIMESLOT_LOST> ID = " + std::to_string(_indeX->GetID()) + " IS NOT READY TO SEND DATA IN #TIMESLOT = " + std::to_string(_TimeSlot));
                        #endif
                        //this->_TimeSlot++;
                    }
                }
                //#ifdef __DEBUG_COMMAND__
                _LogDetail("<DONE> ALL NODES HAVE SENT ALL DATA IN TOTAL " + std::to_string(this->_TimeSlot) + " TIME SLOTS");
                //#endif
                break;
            default:
                _LogError("INVALID INPUT PROTOCOL IN NETWORKSIMULATORENGINE::STARTSIMULATYION(INT)");
                break;
        }
        
    }
    
};



int main(int argc, const char * argv[]) {
    
    /*Node* a = new Node(A, __DEF_A_NODE_DATA__, 5);
    Node* b = new Node(B, __DEF_B_NODE_DATA__, 3);
    Node* c = new Node(C, __DEF_C_NODE_DATA__, 2);
    Node* d = new Node(D, __DEF_D_NODE_DATA__, 4);
    
    std::vector<Node> nodes;
    nodes.push_back(*a);
    nodes.push_back(*b);
    nodes.push_back(*c);
    nodes.push_back(*d);*/
    
    /*std::vector<Node> nodes;
    for(auto i = 1 ; i <= 400; i++)
    {
        if(i <= 100)
            nodes.push_back(Node(i, 110, 5));
        else if(i > 100 && i <= 200)
            nodes.push_back(Node(i, 75, 3));
        else if(i > 200 && i <= 300)
            nodes.push_back(Node(i, 94, 2));
        else
            nodes.push_back(Node(i, 60, 4));
    }*/
    
    /*
    NetworkSimulatorEngin* A = new NetworkSimulatorEngin(nodes);
    A->StartSimulation(CDMA);
    
     */
    return EXIT_SUCCESS;
}
