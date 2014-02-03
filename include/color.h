#ifndef COLOR_H
#define COLOR_H

typedef uint32_t Color;

/** \brief Clamp a value to a given interval. */
template <typename T> inline T
clamp(T value, T min, T max)
{
  return value < min ? min : (value > max ? max : value);
}

/** \brief Get a color from R, G, and B values. */
inline Color
getColorFromRGB (uint8_t r, uint8_t g, uint8_t b)
{
  return static_cast<uint32_t> (r) << 16 |
         static_cast<uint32_t> (g) <<  8 |
         static_cast<uint32_t> (b);
}

/** \brief Get a color from R, G, and B values. */
inline Color
getColorFromRGB (float r, float g, float b)
{
  return getColorFromRGB (static_cast<uint8_t> (std::round (clamp (r, 0.0f, 1.0f) * 255)),
                          static_cast<uint8_t> (std::round (clamp (g, 0.0f, 1.0f) * 255)),
                          static_cast<uint8_t> (std::round (clamp (b, 0.0f, 1.0f) * 255)));
}

/** \brief Get R, G, and B values from a color. */
inline std::tuple<float, float, float>
getRGBFromColor (Color color)
{
  float r = static_cast<float> (((color >> 16) & 0xFF)) / 255.0;
  float g = static_cast<float> (((color >>  8) & 0xFF)) / 255.0;
  float b = static_cast<float> (((color >>  0) & 0xFF)) / 255.0;
  return std::make_tuple (r, g, b);
}

/** \brief Get R, G, and B values as floats from a color. */
inline void
getRGBFromColor (Color color, float* rgb)
{
  rgb[0] = static_cast<float> (((color >> 16) & 0xFF)) / 255.0;
  rgb[1] = static_cast<float> (((color >>  8) & 0xFF)) / 255.0;
  rgb[2] = static_cast<float> (((color >>  0) & 0xFF)) / 255.0;
}

/** \brief Get R, G, and B values as unsigned chars from a color. */
inline void
getRGBFromColor (Color color, uint8_t* rgb)
{
  rgb[0] = (color >> 16) & 0xFF;
  rgb[1] = (color >>  8) & 0xFF;
  rgb[2] = (color >>  0) & 0xFF;
}

/** \brief Generate a random color. */
inline Color
generateRandomColor ()
{
  uint8_t r = static_cast<uint8_t> ((rand () % 256));
  uint8_t g = static_cast<uint8_t> ((rand () % 256));
  uint8_t b = static_cast<uint8_t> ((rand () % 256));
  return getColorFromRGB (r, g, b);
}

/** \brief Get the color for a given number in [0..1] using "jet" colormap. */
inline Color
getJetColor (float value)
{
  float four_value = clamp (value, 0.0f, 1.0f) * 4;
  float r = std::min (four_value - 1.5, -four_value + 4.5);
  float g = std::min (four_value - 0.5, -four_value + 3.5);
  float b = std::min (four_value + 0.5, -four_value + 2.5);
  return getColorFromRGB (r, g, b);
}

/** \brief Get the color for a given number in [0..1] using "autumn" colormap. */
inline Color
getAutumnColor (float value)
{
  return getColorFromRGB (1.0f, value, 0.0f);
}

/** \brief Get the color for a given number in [0..1] using "cool" colormap. */
inline Color
getCoolColor (float value)
{
  return getColorFromRGB (value, 1.0f - value, 1.0f);
}

/** \brief Get the color for a given number in [0..1] using "summer" colormap. */
inline Color
getSummerColor (float value)
{
  return getColorFromRGB (value, (1.0f + value) * 0.5f, 0.4f);
}

#endif /* COLOR_H */

