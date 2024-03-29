/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef COMMAND_PANEL_H
#define COMMAND_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <QGroupBox>
# include <rviz/panel.h>
#endif

class QLineEdit;

namespace panel_test
{


class CommandPanel: public rviz::Panel
{

Q_OBJECT
public:
  CommandPanel( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;
  void sendVel();
  void sendDistance();	

public Q_SLOTS:
  	void setVel(int index);
	void setDistance(int index);
	void setBackDistance(int index);
	void setGo();
	void setStop();

private:
	void createSettings();
	void createButtons();

	QGroupBox *settingsGroup;
	QGroupBox *buttonsGroup;
  	
	ros::NodeHandle nh;
  	ros::Publisher velocity_pub;
	ros::Publisher distance_pub;
        // The latest velocity values from the drive widget.
  	float goal_velocity;
   	float goal_distance;
};

} // end namespace panel_test

#endif // COMMAND_PANEL_H
