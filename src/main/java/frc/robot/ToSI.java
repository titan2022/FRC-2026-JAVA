package frc.robot;

public final class ToSI {
  // SI prefixes
  public static final double exa = 1000000000000000000.0;
  public static final double peta = 1000000000000000.0;
  public static final double tera = 1000000000000.0;
  public static final double giga = 1000000000.0;
  public static final double mega = 1000000.0;
  public static final double kilo = 1000.0;
  public static final double hecto = 100.0;
  public static final double deca = 10.0;
  public static final double deka = 10.0;

  public static final double deci = 0.1;
  public static final double centi = 0.01;
  public static final double milli = 0.001;
  public static final double micro = 0.000001;
  public static final double nano = 0.000000001;
  public static final double pico = 0.000000000001;
  public static final double femto = 0.000000000000001;
  public static final double atto = 0.000000000000000001;

  // Units of time - SI
  public static final double second = 1;
  public static final double s = second;
  public static final double millisecond = milli*second;
  public static final double ms = millisecond;

  // Units of time - international standard
  public static final double minute = 60*s;
  public static final double min = minute;
  public static final double hour = 60*min;
  public static final double hr = 60;
  public static final double day = 24*hour;

  // Units of length - SI
  public static final double metre = 1;
  public static final double m = metre;
  public static final double centimetre = centi*metre;
  public static final double cm = centimetre;
  public static final double millimetre = milli*metre;
  public static final double mm = millimetre;
  public static final double kilometre = kilo*metre;
  public static final double km = kilometre;

  // Units of length - imperial/US customary
  public static final double inch = 0.0254;
  public static final double in = inch;
  public static final double foot = 12*in;
  public static final double ft = foot;
  public static final double yard = 3*ft;
  public static final double yd = yard;
  public static final double mile = 5280*ft;
  public static final double mi = mile;

  // Units of mass - SI
  public static final double kilogram = 1;
  public static final double kg = kilogram;
  public static final double gram = milli*kilogram;
  public static final double g = gram;

  // Units of mass - imperial/US customary
  public static final double pound = 0.45359237*kg;
  public static final double lb = pound;

  // Units of force - helpers;
  public static final double standardGravity = 9.80665 * (m / (s * s));

  // Units of force - SI
  public static final double newton = (kg*m)/(s*s);
  public static final double N = newton;

  // Units of force - other metric
  public static final double kilogramForce = kg * standardGravity;
  public static final double kgf = kilogramForce;

  // Units of force - imperial/US customary
  public static final double poundForce = lb * standardGravity;
  public static final double lbf = poundForce;

  // Now we can define the slug
  public static final double slug = lbf * (s * s) / ft;

  // Units of angle
  public static final double radian = 1;
  public static final double rotation = 2*Math.PI;
  public static final double degree = rotation / 360;
}
