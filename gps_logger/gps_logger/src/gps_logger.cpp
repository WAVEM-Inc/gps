#include"gps_logger.hpp"

GpsLogger::GpsLogger():Node("gps_logger_node"){
	sub_ori_gps_ = this->create_subscription<GpsMSG>("/ublox/fix", 1, std::bind(&GpsLogger::gps_ori_callback ,this ,std::placeholders::_1));
	sub_filter_gps_ = this->create_subscription<GpsMSG>("/gps/filtered", 1, std::bind(&GpsLogger::gps_filter_callback ,this ,std::placeholders::_1));
	char file_name[32];		
	now = time(NULL);
	fix_t = localtime(&now);
	memset(file_name,0,sizeof(file_name));
	sprintf(file_name, "log_gps_ori_%02d%02d%02d%02d%02d",fix_t->tm_mon, fix_t->tm_mday,fix_t->tm_hour,fix_t->tm_min,fix_t->tm_sec);	
	fd_ori_log=open(file_name,O_RDWR|O_CREAT,0777);
	memset(file_name,0,sizeof(file_name));
	sprintf(file_name, "log_gps_ori_%02d%02d%02d%02d%02d.gpx",fix_t->tm_mon, fix_t->tm_mday,fix_t->tm_hour,fix_t->tm_min,fix_t->tm_sec);	
	fd_ori_gpx=open(file_name,O_RDWR|O_CREAT,0777);

	memset(file_name,0,sizeof(file_name));
	sprintf(file_name, "log_gps_filter_%02d%02d%02d%02d%02d",fix_t->tm_mon, fix_t->tm_mday,fix_t->tm_hour,fix_t->tm_min,fix_t->tm_sec);	
	fd_filter_log=open(file_name,O_RDWR|O_CREAT,0777);
	memset(file_name,0,sizeof(file_name));
	sprintf(file_name, "log_gps_filter_%02d%02d%02d%02d%02d.gpx",fix_t->tm_mon, fix_t->tm_mday,fix_t->tm_hour,fix_t->tm_min,fix_t->tm_sec);	
	fd_filter_gpx=open(file_name,O_RDWR|O_CREAT,0777);

}

void GpsLogger::gps_ori_callback(const std::shared_ptr<GpsMSG> fix){
	char fix_log[256];
	now = time(NULL);
	fix_t = localtime(&now);
	memset(fix_log,0,sizeof(fix_log));
	sprintf(fix_log,"\t latitude_x=%f,\t longitude_y=%f,\t time : %02d:%02d:%02d\n" ,fix->latitude, fix->longitude,fix_t->tm_hour,fix_t->tm_min,fix_t->tm_sec );
	write(fd_ori_log,fix_log, strlen(fix_log));
	memset(fix_log,0,sizeof(fix_log));
	sprintf(fix_log,"\n <trkpt lat=\"%f\" lon=\"%f\">\n\n<ele>0</ele>\n\n<time> </time>\n\n</trkpt>\n " ,fix->latitude, fix->longitude);
	write(fd_ori_gpx,fix_log, strlen(fix_log));
}
void GpsLogger::gps_filter_callback(const std::shared_ptr<GpsMSG> fix){
	char fix_log[256];
	now = time(NULL);
	fix_t = localtime(&now);
	memset(fix_log,0,sizeof(fix_log));
	sprintf(fix_log,"\t latitude_x=%f,\t longitude_y=%f,\t time : %02d:%02d:%02d\n" ,fix->latitude, fix->longitude,fix_t->tm_hour,fix_t->tm_min,fix_t->tm_sec );
	write(fd_filter_log,fix_log, strlen(fix_log));
	memset(fix_log,0,sizeof(fix_log));
	sprintf(fix_log,"\n <trkpt lat=\"%f\" lon=\"%f\">\n\n<ele>0</ele>\n\n<time> </time>\n\n</trkpt>\n " ,fix->latitude, fix->longitude);
	write(fd_filter_gpx,fix_log, strlen(fix_log));
}
