#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/bool_property.h>
#include "rviz/properties/color_property.h"
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/vector_property.h>

#include "rigid_body_display.h"
#include <limits>

namespace optitrack
{
RigidBodyDisplay::RigidBodyDisplay()
{
  // Properties
  id_property_ = new rviz::IntProperty( "Body id", 1,
                                        "Rigid body id used by optitrack",
                                        this, SLOT( updateSink() ) );
  diameter_property_ = new rviz::FloatProperty( "Diameter", 6,
                                                "Marker diameter in millimeters",
                                                this, SLOT( updateSink() ) );
  hull_property_ = new rviz::StringProperty( "Convex Hull", "", 
                                              "Convex hull parameters.  (Not editable)",
                                              this);
  show_hull_property_ = new rviz::BoolProperty( "Show", true,
                                                "Whether or not to show the rigid body convex hull.",
                                                hull_property_);
  color_property_ = new rviz::ColorProperty("Color", Qt::magenta,
                                            "Color to draw the convex hull.",
                                            hull_property_);
  // Plane fitting
  plane_property_ = new rviz::StringProperty( "Plane", "", 
                                              "Plane to fit the selected markers.  (Not editable)",
                                              this);
  normal_property_ = new rviz::VectorProperty( "Normal", Ogre::Vector3::UNIT_Y, "", plane_property_);
  show_plane_property_ = new rviz::BoolProperty("Show", false,
                                                "Whether or not to show the fitting plane",
                                                plane_property_);
  selected_property_ = new rviz::IntProperty( "Selected", 0, 
                                              "Number of markers selected.  (Not editable)",
                                              plane_property_);
  fit_property_ = new rviz::BoolProperty( "Fit", false,
                                          "Fit the selected markers to the plane.",
                                          plane_property_);
  // Non editable properties
  hull_property_->setReadOnly( true );
  plane_property_->setReadOnly( true );
  selected_property_->setReadOnly( true );
  // Limits
  id_property_->setMin( 1 );
  id_property_->setMax( 24 );
}

void RigidBodyDisplay::onInitialize()
{
  MFDClass::onInitialize();
  // Hide some controls
  show_plane_property_->hide();
  fit_property_->hide();
}

RigidBodyDisplay::~RigidBodyDisplay()
{
}

void RigidBodyDisplay::reset()
{
  MFDClass::reset();
}

void RigidBodyDisplay::updateSink()
{
}

void RigidBodyDisplay::processMessage( const optitrack::RigidBodyArray::ConstPtr& msg )
{
  // Do nothing
}

} // namespace optitrack

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( optitrack::RigidBodyDisplay, rviz::Display )
