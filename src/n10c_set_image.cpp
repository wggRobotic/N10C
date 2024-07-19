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

static void draw_horizontal_line(unsigned char *pixels, uint32_t width, uint32_t height, uint32_t y, uint32_t radius, uint32_t rgba)
{
	auto top = y - radius;
	auto bottom = y + radius;
	if (top >= height) return;

	bottom = bottom >= height ? height - 1 : bottom;

	auto ptr = (uint32_t *) pixels + top * width;

	for (unsigned i = 0; i < width * (bottom - top); ++i)
	{
		*ptr = rgba;
		++ptr;
	}
}

static void rotate_image_90_clockwise(uint32_t *dest, const uint32_t *src, uint32_t width, uint32_t height)
{
	for (uint32_t y = 0; y < height; ++y)
		for (uint32_t x = 0; x < width; ++x) dest[(height - y - 1) + x * height] = src[x + y * width];
}

static void rotate_image_90_counter_clockwise(uint32_t *dest, const uint32_t *src, uint32_t width, uint32_t height)
{
	for (uint32_t y = 0; y < height; ++y)
		for (uint32_t x = 0; x < width; ++x) dest[y + (width - x - 1) * height] = src[x + y * width];
}

static void rotate_image_180(uint32_t *dest, const uint32_t *src, uint32_t width, uint32_t height)
{
	for (uint32_t y = 0; y < height; ++y)
		for (uint32_t x = 0; x < width; ++x) dest[(width - x - 1) + (height - y - 1) * width] = src[x + y * width];
}

static void rotate_image(uint32_t *dest, const uint32_t *src, uint32_t &width, uint32_t &height, int rotation)
{
	switch (rotation % 4)
	{
	case 1: rotate_image_90_clockwise(dest, src, width, height); break;
	case 2: rotate_image_180(dest, src, width, height); break;
	case 3: rotate_image_90_counter_clockwise(dest, src, width, height); break;
	default: std::memcpy(dest, src, width * height * 4); break;
	}
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

	int rotation = 0;
	switch (index)
	{
	case 0:
		rotation = 2;
		if (m_ActivatedLine) draw_horizontal_line(pixels, width, height, 40, 5, 0xff0000ff); // 40px = 26cm -> wrong: depends on monitor dpi
		break;
	}

	auto output_pixels = new unsigned char[width * height * 4];
	rotate_image((uint32_t *) output_pixels, (const uint32_t *) pixels, width, height, rotation);
	delete[] pixels; // Free the original pixels array -> goddamn it's NOT an array, it's a pointer

	Schedule(
		[this, index, width, height, output_pixels]()
		{
			m_Images[index].StorePixels(width, height, output_pixels);
			delete[] output_pixels;
		});

	// run you fools
}
