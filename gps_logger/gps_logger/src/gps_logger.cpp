#include"gps_logger.hpp"

GpsLogger::GpsLogger():Node("gps_logger_node"){
	sub_ori_gps_ = this->create_subscription<GpsMSG>("/ublox/fix", 1, std::bind(&GpsLogger::gps_ori_callback ,this ,std::placeholders::_1));
	sub_filter_gps_ = this->create_subscription<GpsMSG>("/sensor/ublox/fix", 1, std::bind(&GpsLogger::gps_filter_callback ,this ,std::placeholders::_1));
	sub_imu_gps_ = this->create_subscription<GpsMSG>("/gnss", 1, std::bind(&GpsLogger::gps_imu_callback ,this ,std::placeholders::_1));
	sub_signal_gps_ = this->create_subscription<StrMSG>("/sensor/ublox/log", 1, std::bind(&GpsLogger::gps_signal_callback ,this ,std::placeholders::_1));
}

void GpsLogger::gps_signal_callback(const std::shared_ptr<StrMSG> str)
{
	if(str->data.compare(std::string("START")) ==0 )
	{
		log_num++;
		if(flag_log == true)
		{
			close(fd_filter_gpx);
			close(fd_filter_log);
			close(fd_imu_gpx);
			close(fd_imu_log);
			close(fd_ori_gpx);
			close(fd_ori_log);
		}
		char file_name[64];		
		now = time(NULL);
		fix_t = localtime(&now);
		memset(file_name,0,sizeof(file_name));
		sprintf(file_name, "/home/nuc-bt/gps_log/log_gps_ori_%02d%02d%02d%02d%02d_%02d",fix_t->tm_mon, fix_t->tm_mday,fix_t->tm_hour,fix_t->tm_min,fix_t->tm_sec,log_num);	
		fd_ori_log=open(file_name,O_RDWR|O_CREAT,0777);
		memset(file_name,0,sizeof(file_name));
		sprintf(file_name, "/home/nuc-bt/gps_log/log_gps_ori_%02d%02d%02d%02d%02d_%02d.gpx",fix_t->tm_mon, fix_t->tm_mday,fix_t->tm_hour,fix_t->tm_min,fix_t->tm_sec,log_num);	
		fd_ori_gpx=open(file_name,O_RDWR|O_CREAT,0777);

		memset(file_name,0,sizeof(file_name));
		sprintf(file_name, "/home/nuc-bt/gps_log/log_gps_filter_%02d%02d%02d%02d%02d_%02d",fix_t->tm_mon, fix_t->tm_mday,fix_t->tm_hour,fix_t->tm_min,fix_t->tm_sec,log_num);	
		fd_filter_log=open(file_name,O_RDWR|O_CREAT,0777);
		memset(file_name,0,sizeof(file_name));
		sprintf(file_name, "/home/nuc-bt/gps_log/log_gps_filter_%02d%02d%02d%02d%02d_%02d.gpx",fix_t->tm_mon, fix_t->tm_mday,fix_t->tm_hour,fix_t->tm_min,fix_t->tm_sec,log_num);	
		fd_filter_gpx=open(file_name,O_RDWR|O_CREAT,0777);

		memset(file_name,0,sizeof(file_name));
		sprintf(file_name, "/home/nuc-bt/gps_log/log_gps_imu_%02d%02d%02d%02d%02d_%02d",fix_t->tm_mon, fix_t->tm_mday,fix_t->tm_hour,fix_t->tm_min,fix_t->tm_sec,log_num);	
		fd_imu_log=open(file_name,O_RDWR|O_CREAT,0777);
		memset(file_name,0,sizeof(file_name));
		sprintf(file_name, "/home/nuc-bt/gps_log/log_gps_imu_%02d%02d%02d%02d%02d_%02d.gpx",fix_t->tm_mon, fix_t->tm_mday,fix_t->tm_hour,fix_t->tm_min,fix_t->tm_sec,log_num);	
		fd_imu_gpx=open(file_name,O_RDWR|O_CREAT,0777);

		flag_log=true;
	}
	else if(str->data.compare(std::string("STOP")) ==0 )
	{
		flag_log=false;
		close(fd_filter_gpx);
		close(fd_filter_log);
		close(fd_imu_gpx);
		close(fd_imu_log);
		close(fd_ori_gpx);
		close(fd_ori_log);
	}
}
void GpsLogger::gps_ori_callback(const std::shared_ptr<GpsMSG> fix){
	if(flag_log)
	{
		char fix_log[256];
		now = time(NULL);
		fix_t = localtime(&now);
		memset(fix_log,0,sizeof(fix_log));
		sprintf(fix_log,"\t latitude_x=%.10lf,\t longitude_y=%.9lf,\t time : %02d:%02d:%02d\n" ,fix->latitude, fix->longitude,fix_t->tm_hour,fix_t->tm_min,fix_t->tm_sec );
		write(fd_ori_log,fix_log, strlen(fix_log));
		memset(fix_log,0,sizeof(fix_log));
		sprintf(fix_log,"\n <trkpt lat=\"%.10lf\" lon=\"%.9lf\">\n\n<ele>0</ele>\n\n<time> </time>\n\n</trkpt>\n " ,fix->latitude, fix->longitude);
		write(fd_ori_gpx,fix_log, strlen(fix_log));
	}
}
void GpsLogger::gps_filter_callback(const std::shared_ptr<GpsMSG> fix){
	if(flag_log)
	{
		char fix_log[256];
		now = time(NULL);
		fix_t = localtime(&now);
		memset(fix_log,0,sizeof(fix_log));
		sprintf(fix_log,"\t latitude_x=%.10lf,\t longitude_y=%.9lf,\t time : %02d:%02d:%02d\n" ,fix->latitude, fix->longitude,fix_t->tm_hour,fix_t->tm_min,fix_t->tm_sec );
		write(fd_filter_log,fix_log, strlen(fix_log));
		memset(fix_log,0,sizeof(fix_log));
		sprintf(fix_log,"\n <trkpt lat=\"%.10lf\" lon=\"%.9lf\">\n\n<ele>0</ele>\n\n<time> </time>\n\n</trkpt>\n " ,fix->latitude, fix->longitude);
		write(fd_filter_gpx,fix_log, strlen(fix_log));
	}
}
void GpsLogger::gps_imu_callback(const std::shared_ptr<GpsMSG> fix){
	if(flag_log)
	{
		char fix_log[256];
		now = time(NULL);
		fix_t = localtime(&now);
		memset(fix_log,0,sizeof(fix_log));
		sprintf(fix_log,"\t latitude_x=%.10lf,\t longitude_y=%.9lf,\t time : %02d:%02d:%02d\n" ,fix->latitude, fix->longitude,fix_t->tm_hour,fix_t->tm_min,fix_t->tm_sec );
		write(fd_imu_log,fix_log, strlen(fix_log));
		memset(fix_log,0,sizeof(fix_log));
		sprintf(fix_log,"\n <trkpt lat=\"%.10lf\" lon=\"%.9lf\">\n\n<ele>0</ele>\n\n<time> </time>\n\n</trkpt>\n " ,fix->latitude, fix->longitude);
		write(fd_imu_gpx,fix_log, strlen(fix_log));
	}
}
