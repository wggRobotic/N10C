#include <N10C/n10c.hpp>
#include <cstdint>
#include <iostream>
#include <sensor_msgs/image_encodings.hpp>
#include <string>
#include <vector>
#include <cstring> // Für memcpy

static int convert_to_rgba(unsigned char *dst, const unsigned char *src, size_t count, uint8_t bpp, const std::string &encoding)
{
  auto has_alpha = sensor_msgs::image_encodings::hasAlpha(encoding);
  auto is_rgb = encoding == sensor_msgs::image_encodings::RGB8 || encoding == sensor_msgs::image_encodings::RGBA8;
  auto is_bgr = encoding == sensor_msgs::image_encodings::BGR8 || encoding == sensor_msgs::image_encodings::BGRA8;

  if (!is_rgb && !is_bgr)
  {
    std::cerr << "Cannot convert unsupported encoding '" << encoding << "' to RGBA" << std::endl;
    return 1;
  }

  for (size_t i = 0; i < count; ++i)
  {
    auto pi = i * 4;
    auto di = i * bpp;
    dst[pi + 0] = is_rgb ? src[di + 0] : src[di + 2];
    dst[pi + 1] = is_rgb ? src[di + 1] : src[di + 1];
    dst[pi + 2] = is_rgb ? src[di + 2] : src[di + 0];
    dst[pi + 3] = has_alpha ? src[di + 3] : 0xff;
  }

  return 0;
}

static void draw_horizontal_line(unsigned char *pixels, uint32_t width, uint32_t height, uint32_t y, uint32_t radius, uint32_t rgba)
{
  auto top = y - radius;
  auto bottom = y + radius;
  if (bottom < 0 || top >= height) return;

  top = top < 0 ? 0 : top;
  bottom = bottom >= height ? height - 1 : bottom;

  auto ptr = (uint32_t *)pixels + top * width;

  for (unsigned i = 0; i < width * (bottom - top); ++i)
  {
    *ptr = rgba;
    ++ptr;
  }
}

static void rotate_image_90_clockwise(unsigned char* src, unsigned char* dst, uint32_t width, uint32_t height)
{
    for (uint32_t y = 0; y < height; ++y)
    {
        for (uint32_t x = 0; x < width; ++x)
        {
            for (int c = 0; c < 4; ++c)
            {
                dst[(x * height + (height - y - 1)) * 4 + c] = src[(y * width + x) * 4 + c];
            }
        }
    }
}

static void rotate_image(unsigned char* src, unsigned char* dst, uint32_t& width, uint32_t& height, int rotations)
{
    rotations = rotations % 4; // Normalize the number of rotations
    if (rotations == 0) {
        std::memcpy(dst, src, width * height * 4);
        return;
    }
    
    unsigned char* temp_src = src;
    unsigned char* temp_dst = dst;
    unsigned char* temp_buf = new unsigned char[width * height * 4];
    uint32_t temp_width = width;
    uint32_t temp_height = height;

    for (int i = 0; i < rotations; ++i)
    {
        rotate_image_90_clockwise(temp_src, temp_dst, temp_width, temp_height);
        std::swap(temp_src, temp_dst);
        std::swap(temp_width, temp_height);
    }

    if (temp_src != dst) {
        std::memcpy(dst, temp_src, width * height * 4);
    }

    width = temp_width;
    height = temp_height;
    delete[] temp_buf;
}

void N10C::SetImage(size_t index, const std::vector<uint8_t> &data, uint32_t width, uint32_t height, uint32_t step, const std::string &encoding)
{
  if (!IsActive()) return;

  auto bpp = step / width;
  auto pixels = new unsigned char[width * height * 4];
  auto output_pixels = new unsigned char[width * height * 4];

  if (convert_to_rgba(pixels, data.data(), width * height, bpp, encoding))
  {
    delete[] pixels;
    delete[] output_pixels;
    return;
  }

  

  

  int rotations = 0;
  if (index == 4)
  {
    rotations = 3; // Example: Rotate 180 degrees
  }

  if(index == 0){
    rotations = 2;
  }
  if (index == 0 && m_ActivatedLine)
  { //40px = 26cm
    draw_horizontal_line(pixels, width, height,  40, 5, 0xff0000ff);
  }

  rotate_image(pixels, output_pixels, width, height, rotations);

  Schedule(
      [this, index, width, height, output_pixels]()
      {
          m_Images[index].StorePixels(width, height, output_pixels);
          delete[] output_pixels;
      });

  delete[] pixels; // Free the original pixels array
}
