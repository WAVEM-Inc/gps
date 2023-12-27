#ifndef GPS_IAO_DOOR_DETERMINATION__GPS_DATA__HPP_
#define GPS_IAO_DOOR_DETERMINATION__GPS_DATA__HPP_
namespace GPS{
    class Data{
        private : 
            float latitude_;
            float longitude_;
        public :
            Data(float latitude,float longitude): latitude_(latitude), longitude_(longitude){
                
            }
            const float get_latitude(){
                return latitude_;
            }
            const float get_logitude(){
                return longitude_;
            }
    };  
}
#endif