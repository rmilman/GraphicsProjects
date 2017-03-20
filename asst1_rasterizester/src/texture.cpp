#include "texture.h"
#include "CGL/color.h"


namespace CGL {

// Examines the enum parameters in sp and performs
// the appropriate sampling using the three helper functions below.
Color Texture::sample(const SampleParams &sp) {
 
  // Part 6: Fill in the functionality for sampling 
  //          nearest or bilinear in mipmap level 0, conditional on sp.psm
 // cout << level << endl;
  int level = get_level(sp);
  if (sp.psm == 0) { //pnearest
    if (sp.lsm == 0){
      return sample_nearest(sp.uv, 0);
    }else if (sp.lsm == 1){
      return sample_nearest(sp.uv, level);
    }else if (sp.lsm == 2){
      int lowLevel = (int) floor(level);
      Color u0 = (level-lowLevel)*sample_nearest(sp.uv, lowLevel) + (lowLevel+1-level)*sample_nearest(sp.uv, lowLevel+1);
      return u0;
    }
  } else {
    if (sp.lsm == 0){
      return sample_bilinear(sp.uv, 0);
    }else if (sp.lsm == 1){
      return sample_bilinear(sp.uv, level);
    }else if (sp.lsm == 2) {
      int lowLevel = (int) floor(level);
      Color u0 = (level-lowLevel)*sample_bilinear(sp.uv, lowLevel) + (lowLevel+1-level)*sample_bilinear(sp.uv,lowLevel+1);
      return u0;
    }
  }
 
  // Part 7: Fill in full sampling (including trilinear), 
  //          conditional on sp.psm and sp.lsm
  
  return Color();
}

// Given sp.du and sp.dv, returns the appropriate mipmap
// level to use for L_NEAREST or L_LINEAR filtering.
float Texture::get_level(const SampleParams &sp) {
 // Matrix2D scale = Matrix2D(width, height);
  Vector2D tuv = Vector2D(sp.uv[0]*width, sp.uv[1]*height);
  Vector2D tdu = Vector2D(sp.du[0]*width, sp.du[1]*height);
  Vector2D tdv = Vector2D(sp.dv[0]*width, sp.dv[1]*height);
    
  float L = max((tdu-tuv).norm(),(tdv-tuv).norm());

  L = log2f(L);
  if (L<0){
    L=0;
  }else if (L>=kMaxMipLevels){
    L=kMaxMipLevels;
  }
  return L;
}

// Indexes into the level'th mipmap
// and returns the nearest pixel to (u,v)
Color Texture::sample_nearest(Vector2D uv, int level) {
    int widthm = mipmap[level].width; 
    int heightm = mipmap[level].height;
    int u = uv[0]*widthm;
    int v = uv[1]*heightm;
    if (u>=widthm){
      u=widthm-1;
    } else if (v>=heightm){
      v=heightm-1;
    }
    int i = floor(4*(widthm*v+u));
    //cout << u << "u  v" << v << endl;
    return Color(&mipmap[level].texels[i]);
}

// Indexes into the level'th mipmap
// and returns a bilinearly weighted combination of
// the four pixels surrounding (u,v)
Color Texture::sample_bilinear(Vector2D uv, int level) {
  int widthm = mipmap[level].width; 
  int heightm = mipmap[level].height;
  float u = uv[0]*widthm;
  float v = uv[1]*heightm;
   if (u>=widthm){
      u=widthm-1;
    } else if (v>=heightm){
      v=heightm-1;
    }
  int ulow = floor(u);
  int uhigh = ulow+1;
  int vlow = floor(v);
  int vhigh = vlow+1;
  float udif = u + (-1) * ulow;
  float vdif = v + (-1) * vlow;

  //edge cases
  if (uhigh >= widthm || vhigh >= heightm){
     if (uhigh >= widthm && vhigh >= heightm){
      return (Color(&mipmap[level].texels[floor(4*(ulow+vlow*widthm))]));
     }else if (uhigh >= widthm){
      Color u00 = Color(&mipmap[level].texels[floor(4*(ulow+vlow*widthm))]);
      Color u01 = Color(&mipmap[level].texels[floor(4*(ulow+vhigh*widthm))]);
      Color u0 = u00 + vdif*(u01 + -1 * u00);
      return u0;
     }else{
      Color u00 = Color(&mipmap[level].texels[floor(4*(ulow+vlow*widthm))]);
      Color u10 = Color(&mipmap[level].texels[floor(4*(uhigh+vlow*widthm))]);
      Color u0 = u00 + udif*(u10 + -1 * u00);
      return u0;
     }
  }else{

  Color u00 = Color(&mipmap[level].texels[floor(4*(ulow+vlow*widthm))]);
  Color u01 = Color(&mipmap[level].texels[floor(4*(ulow+vhigh*widthm))]);
  Color u10 = Color(&mipmap[level].texels[floor(4*(uhigh+vlow*widthm))]);
  Color u11 = Color(&mipmap[level].texels[floor(4*(uhigh+vhigh*widthm))]);
  
  Color u0 = u00 + udif*(u10 + -1 * u00);
  Color u1 = u01 + udif*(u11 + -1 * u01);
  Color uf = u0 + vdif*(u1 + -1 * u0);
  return uf;
}
}



/****************************************************************************/



inline void uint8_to_float(float dst[4], unsigned char *src) {
  uint8_t *src_uint8 = (uint8_t *)src;
  dst[0] = src_uint8[0] / 255.f;
  dst[1] = src_uint8[1] / 255.f;
  dst[2] = src_uint8[2] / 255.f;
  dst[3] = src_uint8[3] / 255.f;
}

inline void float_to_uint8(unsigned char *dst, float src[4]) {
  uint8_t *dst_uint8 = (uint8_t *)dst;
  dst_uint8[0] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[0])));
  dst_uint8[1] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[1])));
  dst_uint8[2] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[2])));
  dst_uint8[3] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[3])));
}

void Texture::generate_mips(int startLevel) {

  // make sure there's a valid texture
  if (startLevel >= mipmap.size()) {
    std::cerr << "Invalid start level";
  }

  // allocate sublevels
  int baseWidth = mipmap[startLevel].width;
  int baseHeight = mipmap[startLevel].height;
  int numSubLevels = (int)(log2f((float)max(baseWidth, baseHeight)));

  numSubLevels = min(numSubLevels, kMaxMipLevels - startLevel - 1);
  mipmap.resize(startLevel + numSubLevels + 1);

  int width = baseWidth;
  int height = baseHeight;
  for (int i = 1; i <= numSubLevels; i++) {

    MipLevel &level = mipmap[startLevel + i];

    // handle odd size texture by rounding down
    width = max(1, width / 2);
    //assert (width > 0);
    height = max(1, height / 2);
    //assert (height > 0);

    level.width = width;
    level.height = height;
    level.texels = vector<unsigned char>(4 * width * height);
  }

  // create mips
  int subLevels = numSubLevels - (startLevel + 1);
  for (int mipLevel = startLevel + 1; mipLevel < startLevel + subLevels + 1;
       mipLevel++) {

    MipLevel &prevLevel = mipmap[mipLevel - 1];
    MipLevel &currLevel = mipmap[mipLevel];

    int prevLevelPitch = prevLevel.width * 4; // 32 bit RGBA
    int currLevelPitch = currLevel.width * 4; // 32 bit RGBA

    unsigned char *prevLevelMem;
    unsigned char *currLevelMem;

    currLevelMem = (unsigned char *)&currLevel.texels[0];
    prevLevelMem = (unsigned char *)&prevLevel.texels[0];

    float wDecimal, wNorm, wWeight[3];
    int wSupport;
    float hDecimal, hNorm, hWeight[3];
    int hSupport;

    float result[4];
    float input[4];

    // conditional differentiates no rounding case from round down case
    if (prevLevel.width & 1) {
      wSupport = 3;
      wDecimal = 1.0f / (float)currLevel.width;
    } else {
      wSupport = 2;
      wDecimal = 0.0f;
    }

    // conditional differentiates no rounding case from round down case
    if (prevLevel.height & 1) {
      hSupport = 3;
      hDecimal = 1.0f / (float)currLevel.height;
    } else {
      hSupport = 2;
      hDecimal = 0.0f;
    }

    wNorm = 1.0f / (2.0f + wDecimal);
    hNorm = 1.0f / (2.0f + hDecimal);

    // case 1: reduction only in horizontal size (vertical size is 1)
    if (currLevel.height == prevLevel.height) {
      //assert (currLevel.height == 1);

      for (int i = 0; i < currLevel.width; i++) {
        wWeight[0] = wNorm * (1.0f - wDecimal * i);
        wWeight[1] = wNorm * 1.0f;
        wWeight[2] = wNorm * wDecimal * (i + 1);

        result[0] = result[1] = result[2] = result[3] = 0.0f;

        for (int ii = 0; ii < wSupport; ii++) {
          uint8_to_float(input, prevLevelMem + 4 * (2 * i + ii));
          result[0] += wWeight[ii] * input[0];
          result[1] += wWeight[ii] * input[1];
          result[2] += wWeight[ii] * input[2];
          result[3] += wWeight[ii] * input[3];
        }

        // convert back to format of the texture
        float_to_uint8(currLevelMem + (4 * i), result);
      }

      // case 2: reduction only in vertical size (horizontal size is 1)
    } else if (currLevel.width == prevLevel.width) {
      //assert (currLevel.width == 1);

      for (int j = 0; j < currLevel.height; j++) {
        hWeight[0] = hNorm * (1.0f - hDecimal * j);
        hWeight[1] = hNorm;
        hWeight[2] = hNorm * hDecimal * (j + 1);

        result[0] = result[1] = result[2] = result[3] = 0.0f;
        for (int jj = 0; jj < hSupport; jj++) {
          uint8_to_float(input, prevLevelMem + prevLevelPitch * (2 * j + jj));
          result[0] += hWeight[jj] * input[0];
          result[1] += hWeight[jj] * input[1];
          result[2] += hWeight[jj] * input[2];
          result[3] += hWeight[jj] * input[3];
        }

        // convert back to format of the texture
        float_to_uint8(currLevelMem + (currLevelPitch * j), result);
      }

      // case 3: reduction in both horizontal and vertical size
    } else {

      for (int j = 0; j < currLevel.height; j++) {
        hWeight[0] = hNorm * (1.0f - hDecimal * j);
        hWeight[1] = hNorm;
        hWeight[2] = hNorm * hDecimal * (j + 1);

        for (int i = 0; i < currLevel.width; i++) {
          wWeight[0] = wNorm * (1.0f - wDecimal * i);
          wWeight[1] = wNorm * 1.0f;
          wWeight[2] = wNorm * wDecimal * (i + 1);

          result[0] = result[1] = result[2] = result[3] = 0.0f;

          // convolve source image with a trapezoidal filter.
          // in the case of no rounding this is just a box filter of width 2.
          // in the general case, the support region is 3x3.
          for (int jj = 0; jj < hSupport; jj++)
            for (int ii = 0; ii < wSupport; ii++) {
              float weight = hWeight[jj] * wWeight[ii];
              uint8_to_float(input, prevLevelMem +
                                        prevLevelPitch * (2 * j + jj) +
                                        4 * (2 * i + ii));
              result[0] += weight * input[0];
              result[1] += weight * input[1];
              result[2] += weight * input[2];
              result[3] += weight * input[3];
            }

          // convert back to format of the texture
          float_to_uint8(currLevelMem + currLevelPitch * j + 4 * i, result);
        }
      }
    }
  }
}

}