#pragma once

// Menu for MapPoint

// 	name: 					(anything)				writeable
//  circle radius:			(4 pixels)				writeable
// 	circle color:			(shows color)			color menu
// 	lat and lon: 			(64.5325, 32.9724) 		writeable
// 	MGRS:					(35W CP 87156 43789)	writeable
//  elevation:				(434 meters)			
// 	last update 			(hh:mm dd.mm.yyyy)
// 	show viewbale area		(on/off)				toggle button
// 		use restrictions		(on/off)				toggle burron
// 			right restriction 		(degrees)				writeable
// 			left restriction 		(degrees)				writeable
// 			distance restriction 	(meters)				writeable
// 		viewable area color		(shows color)			color menu
 

class MapPoint {
public:
	char*			m_name{nullptr};
	char*			m_timeLastUpdate{nullptr};
	
	double 			m_lat=0.0f, 
					m_lon=0.0f;
	
	unsigned int 	m_rightRestriction_degrees=361, // 361 not possible
					m_leftRestriction_degrees=361,  // 361 not possible
					m_distanceRestriction_meters=0; 
					
	unsigned int	m_elevationMeters=0;
	unsigned int 	m_circleRadiusPixels=0;
	
	unsigned char 	m_pointRed,   m_viewableAreaRed,
					m_pointGreen, m_viewableAreaGreen,
					m_pointBlue,  m_viewableAreaBlue;
	
	bool 			m_showViewableArea=false;
	bool			m_usingRestrictions=false;
	bool 			m_isInView=false;
	bool 			m_isVisible=false;
	bool			m_isHovered=false;
	
	
	// Constructor
					MapPoint(double lat, 
							 double lon, 
							 char* name, 
							 unsigned int elevationMeters, 
							 bool showViewableArea,
							 unsigned char red, 
							 unsigned char green, 
							 unsigned char blue);
	
	
	void 		 	getMGRS(char* returnString);
	void 			setMGRS(char* inputString);
	void 			getLatLon(double* returnLat, double* returnLon);
	void 			setLatLon(double lat, double lon);
	void			setPointColor		(unsigned char  red, unsigned char  green, unsigned char  blue);
	void			getPointColor		(unsigned char* red, unsigned char* green, unsigned char* blue);
	void			setViewableAreaColor(unsigned char  red, unsigned char  green, unsigned char  blue);
	void			getViewableAreaColor(unsigned char* red, unsigned char* green, unsigned char* blue);
	
	void 			setRestrictions(bool on, float rightRestriction_degrees=0, float leftRestriction_degrees=0, unsigned int distanceRestriction=0);
									
									
	void			draw(unsigned char* imageData, int imageWidth, int imageHeight, int nrChannels);
	
private:
	int 			m_xPixel, 
					m_yPixel;
	
	void 			set 
	void 			setLastUpdate();
};