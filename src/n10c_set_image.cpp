#include <N10C/n10c.hpp>
#include <sensor_msgs/image_encodings.hpp>

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

void N10C::SetImage(size_t index, const std::vector<uint8_t> &data, uint32_t width, uint32_t height, uint32_t step, const std::string &encoding)
{
  if (!IsActive()) return;

  auto bpp = step / width;
  auto pixels = new unsigned char[width * height * 4];

  if (convert_to_rgba(pixels, data.data(), width * height, bpp, encoding))
  {
    delete[] pixels;
    return;
  }

  Schedule(
      [this, index, width, height, pixels]()
      {
        m_Images[index].StorePixels(width, height, pixels);
        delete[] pixels;
      });
}
