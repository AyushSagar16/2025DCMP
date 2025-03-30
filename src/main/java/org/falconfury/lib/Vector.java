package org.falconfury.lib;

public class Vector {

  // Variables for initialization
  double x_component_;
  double y_component_;

  // Constructor
  public Vector(double x_component, double y_component) {
    // Initializing variables
    x_component_ = x_component;
    y_component_ = y_component;
  }

  // Getter methods
  public double getXComponent() {
    return x_component_;
  }

  public double getYComponent() {
    return y_component_;
  }
  
  public double getAngle() {
    return Math.atan2(y_component_, x_component_);
  }

  public double getVectorMagnitude() {
    return Math.sqrt(x_component_ * x_component_ + y_component_ * y_component_);
  }

  public Vector add(Vector other) {
    return new Vector(
      this.x_component_ + other.getXComponent(), 
      this.y_component_ + other.getYComponent());
  }

  public Vector subtract(Vector other) {
    return new Vector(
      this.x_component_ - other.getXComponent(), 
      this.y_component_ - other.getYComponent());
  }

  public void setXComponent(double x_component) {
    x_component_ = x_component;
  }

  public void setYComponent(double y_component) {
    y_component_ = y_component;
  }

  @Override
  public String toString() {
    return String.format("Vector(%.2f, %.2f)", x_component_, y_component_);
  }
}
