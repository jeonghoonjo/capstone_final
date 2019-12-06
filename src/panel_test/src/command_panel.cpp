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

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QComboBox>
#include <QGroupBox>
#include <QPushButton>
#include <QString>


#include <std_msgs/Float64.h>
#include "command_panel.h"

namespace panel_test
{

// BEGIN_TUTORIAL
// Here is the implementation of the TeleopPanel class.  TeleopPanel
// has these responsibilities:
//
// - Act as a container for GUI elements DriveWidget and QLineEdit.
// - Publish command velocities `10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
CommandPanel::CommandPanel( QWidget* parent )
  : rviz::Panel( parent )
  , goal_velocity( 60 )
  , goal_distance( 10 )
{
	createSettings();
	createButtons();

	QHBoxLayout *layout = new QHBoxLayout;
	layout->addWidget(settingsGroup);
	layout->addWidget(buttonsGroup);
	setLayout(layout);
	
	
	velocity_pub = nh.advertise<std_msgs::Float64>("goal_vel", 1000);
	distance_pub = nh.advertise<std_msgs::Float64>("goal_dist", 1000);

}


void CommandPanel::createSettings()
{
	velocity_pub = nh.advertise<std_msgs::Float64>("goal_vel", 1000);
	settingsGroup = new QGroupBox(tr("Settings"));
	
	QLabel *setSpeedLabel = new QLabel(tr("Go backward [cm] :"));
	QComboBox *setSpeedComboBox = new QComboBox;	
	setSpeedComboBox->addItem("5");
	setSpeedComboBox->addItem("10");
	setSpeedComboBox->addItem("15");
	setSpeedComboBox->addItem("20");

	connect(setSpeedComboBox, SIGNAL(activated(int)), this, SLOT(setBackDistance(int)));	

	QLabel *setDistanceLabel = new QLabel(tr("Go forward [cm] :"));
	QComboBox *setDistanceComboBox = new QComboBox;
	setDistanceComboBox->addItem("5");
	setDistanceComboBox->addItem("10");
	setDistanceComboBox->addItem("15");
	setDistanceComboBox->addItem("20");
	
	connect(setDistanceComboBox, SIGNAL(activated(int)), this, SLOT(setDistance(int)));

	QVBoxLayout *settingsLayout = new QVBoxLayout;
	settingsLayout->addWidget(setDistanceLabel);
	settingsLayout->addWidget(setDistanceComboBox);
	settingsLayout->addWidget(setSpeedLabel);
	settingsLayout->addWidget(setSpeedComboBox);
	settingsGroup->setLayout(settingsLayout);
}

void CommandPanel::createButtons()
{
	buttonsGroup = new QGroupBox(tr("Buttons"));

	QPushButton *goButton = new QPushButton(tr("&Go home"));
	connect(goButton, SIGNAL(clicked()), this, SLOT(setGo()));	

	QPushButton *stopButton = new QPushButton(tr("&Stop"));
	connect(stopButton, SIGNAL(clicked()), this, SLOT(setStop()));	

	QPushButton *startInspectbutton = new QPushButton(tr("&Inspection start"));

	QVBoxLayout *buttonsLayout = new QVBoxLayout;
	buttonsLayout->addWidget(goButton);
	buttonsLayout->addWidget(stopButton);
	buttonsLayout->addWidget(startInspectbutton);
	buttonsLayout->addStretch(1);
	buttonsGroup->setLayout(buttonsLayout);

}
void CommandPanel::sendVel()
{
	if(ros::ok() && velocity_pub){
		std_msgs::Float64 msg;
		msg.data = goal_velocity;
		velocity_pub.publish(msg);
		ros::spinOnce();
	}
}

void CommandPanel::sendDistance()
{
	if(ros::ok() && distance_pub){
		std_msgs::Float64 msg;
		msg.data = goal_distance;
		distance_pub.publish(msg);
		ros::spinOnce();
	}
}

void CommandPanel::setBackDistance(int index)
{
	goal_distance = (-1)*(index+1)*1087;
	CommandPanel::sendDistance();
}

void CommandPanel::setDistance(int index)
{
	goal_distance = (index+1)*1087;
	CommandPanel::sendDistance();
}

void CommandPanel::setGo()
{
	goal_distance = (-1)*4*1087;
	CommandPanel::sendDistance();
	
}

void CommandPanel::setStop()
{
	//CommandPanel::setVel("0");	

}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void CommandPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

// Load all configuration data for this panel from the given Config object.
void CommandPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
}

} // end namespace panel_test

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(panel_test::CommandPanel,rviz::Panel )
// END_TUTORIAL
