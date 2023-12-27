#ifndef GPS_IAO_DOOR_DETERMINATION__CONSTANT__CONSTANTS__HPP_
#define GPS_IAO_DOOR_DETERMINATION__CONSTANT__CONSTANTS__HPP_  

/**
 * TEST 0 - develop(complete), 1 - develop(log x) , 2 - develop(log o)
*/
#define TEST 1
#define UBLOX "/ublox/fix"
#define IAO "/in_out_door/determination"
#include <iostream>
#define QUEUE_SIZE 3
namespace Constants{
    class Topic{
        public :
            Topic(): name_ublox_(UBLOX),name_iao_(IAO){

            }
        const std::string name_ublox_;
        const std::string name_iao_;
    };
    class DeterminationData{
        public :
            DeterminationData(): que_size_(QUEUE_SIZE){}
        const int que_size_;
    };
    
}

#endif