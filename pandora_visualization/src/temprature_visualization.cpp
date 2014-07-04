#include "ros/ros.h"

#include <sensor_msgs/Image.h>

class TempratureVisualization

{

  public: 
    
    TempratureVisualization ( int lowTemp , int highTemp ) ; 
    
    void ThermalCallback ( const sensor_msgs ::Image & msg ) ; 
    
    void ConvertImage ( const sensor_msgs ::Image & msg , 
                        sensor_msgs ::Image & image ) ; 
                        
  private: 

    ros ::NodeHandle nh_ ; 

    ros ::Publisher pub_left_ ; 
    ros ::Publisher pub_middle_ ; 
    ros ::Publisher pub_right_ ; 
    
    ros ::Subscriber sub_ ; 
    
    const int lowTemp_ ; 
    const int highTemp_ ; 
    
} ; 

TempratureVisualization ::TempratureVisualization ( int lowTemp , 
                                                    int highTemp ) : 
                                                    
  lowTemp_ ( lowTemp ) , 
  highTemp_ ( highTemp ) 
  
{ 

  pub_left_ = 
  nh_ .advertise < sensor_msgs ::Image > ( "/sensors/thermal_viz/left" , 1 ) ; 

  pub_middle_ = 
  nh_ .advertise < sensor_msgs ::Image > ( "/sensors/thermal_viz/middle" , 1 ) ; 

  pub_right_ = 
  nh_ .advertise < sensor_msgs ::Image > ( "/sensors/thermal_viz/right" , 1 ) ; 
  
  sub_ = nh_ .subscribe ( "/sensors/thermal" , 6 , 
                          & TempratureVisualization ::ThermalCallback , this ) ; 
  
}

void TempratureVisualization ::ThermalCallback ( const sensor_msgs 
                                                        ::Image & msg ) { 
                                                        
  ROS_INFO_THROTTLE 
  ( 5 , "Received a thermal image! Conversion to RGB is starting." ) ; 

  sensor_msgs ::Image image ; 
  
  image .header .stamp = ros:: Time:: now ( ) ; 
  image .header .frame_id = msg .header .frame_id ; 
  image .height = msg .height ; 
  image .width = msg .width ; 
  image .step = msg .width * 3 ; 
  image .encoding = "bgr8" ; 
  
  TempratureVisualization ::ConvertImage ( msg , image ) ; 
  
  if ( image .header .frame_id [ 0 ] == 'l' ) 
  
    this ->pub_left_ .publish ( image ) ; 
  
  else if ( image .header .frame_id [ 0 ] == 'm' ) 
  
    this ->pub_middle_ .publish ( image ) ; 
  
  else if ( image .header .frame_id [ 0 ] == 'r' ) 
  
    this ->pub_right_ .publish ( image ) ; 
  
}

void TempratureVisualization ::ConvertImage ( const sensor_msgs ::Image & msg , 
                                              sensor_msgs ::Image & image ) { 
                 
  //FIXME: Receive from param                             
  float alpha = 0.784 ; 
  
  for ( unsigned int i = 0 ; i < msg .width ; i++ ) { 

    for ( unsigned int j = 0 ; j < msg .height ; j++ ) { 
      
      int temp = msg .data [ i * msg .height + j ] ; 
      
      temp =  std ::min ( std ::max ( temp , lowTemp_ ) , highTemp_ ) ; 
      
      float low_spec = lowTemp_ + ( highTemp_ - lowTemp_ ) * ( 1.0 / 3.0 ) ; 
      float high_spec = lowTemp_ + ( highTemp_ - lowTemp_ ) * ( 2.0 / 3.0 ) ; 
      
      float from , to ; 
      
      float R , G , B ; 
      
      if ( temp < low_spec ) { 
      
        from = 0.5 ; 
        
        to = 1.0 ; 
        
        B = 1.0 ; 
        
        G = 0.0 ; 
      
        R = ( temp - lowTemp_ ) * ( to - from ) / ( highTemp_ - lowTemp_ ) + 
            from ; 
      
      }
      
      else if ( low_spec <= temp && temp <= high_spec ) { 
      
        from = 1.0 ; 
        
        to = 0.0 ; 
        
        B = 1 - ( temp - lowTemp_ ) * ( from - to ) / ( highTemp_ - lowTemp_ ) - 
            from ;
        
        G = 0.0 ; 
      
        R = 1.0 ; 
      
      }
      
      else if ( high_spec < temp ) { 
      
        from = 0.0 ; 
        
        to = 0.5 ; 
        
        B = 0.0 ; 
        
        G = ( temp - lowTemp_ ) * ( to - from ) / ( highTemp_ - lowTemp_ ) + 
            from ;
      
        R = 1.0 ; 
      
      }
      
      float red = ( temp - lowTemp_ ) * 255.0 / ( highTemp_ - lowTemp_ ) ; 
      
      float blue = 1 - red ; 
    
      image .data .push_back ( ( char ) ( B * 255.0 * alpha ) ) ; 
      image .data .push_back ( ( char ) ( G * 255.0 * alpha ) ) ; 
      image .data .push_back ( ( char ) ( R * 255.0 * alpha ) ) ; 
    
    }
    
  }
  
}

int main ( int argc , char ** argv )

{

  ros ::init ( argc , argv , "temprature_visualization" ) ; 
  
  ROS_INFO ( "Starting temprature visualization node." ) ; 
  
  //Grid-eye sensor limits
  const int minTemp = - 20 ; 
  const int maxTemp = 80 ; 
  
  //TODO: Take temprature limits from dynamic reconfigure
  int lowTemp = 20 ; 
  int highTemp = 47 ; 
  
  if ( argc == 3 ) { 
  
    lowTemp = atoi ( argv [ 1 ] ) ; 
    
    lowTemp = std ::max ( minTemp , lowTemp ) ; 
    
    highTemp = atoi ( argv [ 2 ] ) ; 
    
    highTemp = std ::min ( highTemp , maxTemp ) ; 
  
    ROS_INFO ( "Temprature range was set to [%d,%d]C ." , 
               lowTemp , 
               highTemp ) ; 
    
  }
    
  else { 
  
    ROS_INFO ( "Temprature range was not specified, defaults to [%d,%d]C ." , 
               lowTemp , 
               highTemp ) ; 
  
  }

  TempratureVisualization temp_viz ( lowTemp , highTemp ) ; 
  
  ROS_INFO ( "Waiting for thermal messages..." ) ; 

  ros ::spin ( ) ; 

  return 0 ; 
  
}
