#include "Determination.hpp"

GPS::Determination::Determination() : Node("gps_iao_door_determination_node"){
#if TEST == 1
    std::cout<<"start"<<std::endl;
#endif
    constant_topic_= std::make_unique<Constants::Topic>();
    constants_data_ = std::make_unique<Constants::DeterminationData>();
    sub_ublox_ = this->create_subscription<GpsMsg>(constant_topic_->name_ublox_,1,
                                                                    std::bind(&GPS::Determination::sub_ublox_callback,this,std::placeholders::_1));
    pub_iao_ = this->create_publisher<IAOMsg>(constant_topic_->name_iao_,1);
    
}


void GPS::Determination::sub_ublox_callback(const GpsMsg::SharedPtr gps){
#if TEST == 1
    std::cout<<"callback okay"<<std::endl;
#endif

    std::shared_ptr<GPS::Data> gps_data = std::make_shared<GPS::Data>(gps->latitude,gps->longitude);
    std::shared_ptr<IAOMsg> temp_pub = std::make_shared<IAOMsg>();
    int check = iao_check(gps_data);
    limited_size_deque(gps_data);
    if(check== -1){
        return;
    }
    else if(check==1){
        temp_pub->determination = false; 
    }
    else if(check == 0){
        temp_pub->determination = true;
    }
    
    pub_iao_->publish(*temp_pub);
}

void  GPS::Determination::limited_size_deque(const std::shared_ptr<GPS::Data> gps_data){
#if TEST == 1
    std::cout<<"deque input"<<std::endl;
#endif
        gps_deque_.push_back(*gps_data);
        // 크기가 제한을 초과하면 앞에서부터 요소 삭제
        while (gps_deque_.size() > constants_data_->que_size_) {
            gps_deque_.pop_front();
        }
}

int GPS::Determination::iao_check(const std::shared_ptr<GPS::Data> gps_data){
#if TEST == 1
    std::cout<<"iao check"<<std::endl;
#endif
    int lp = 0;
    const int deque_size = gps_deque_.size();
    if(constants_data_->que_size_>deque_size){
#if TEST == 1
    std::cout<<"iao -1"<<std::endl;
#endif
        return -1;
    }

    while(lp<deque_size){
        if(gps_deque_[lp].get_latitude()!=gps_data->get_latitude() || gps_deque_[lp].get_logitude()!=gps_data->get_logitude()){
#if TEST == 1
    std::cout<<"iao 0"<<std::endl;
#endif            
            return 0;
        }
        lp++;
    }

#if TEST == 1
    std::cout<<"iao 1"<<std::endl;
#endif
    return 1;
}
