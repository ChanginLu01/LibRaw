/* -*- C++ -*-
 * File: bayer_merge.cpp
 * Copyright 2025 LibRaw LLC (info@libraw.org)
 *
 * LibRaw sample
 * Merges the 4 subpixels of every Bayer 2x2 CFA block into one monochrome
 * pixel, so the result holds 1/4 of the source pixels (half width, half
 * height). Averaging is done in 32-bit unsigned integer arithmetics (or in
 * double precision, with -f); the image is written as a 16-bit TIFF
 * (default) or as a 16-bit PGM (-p).
 * Files may be listed on the command line, or a whole directory of raw
 * files may be processed at once with --indir. Files are processed one
 * after another, in a single thread.
 *

LibRaw is free software; you can redistribute it and/or modify
it under the terms of the one of two licenses as you choose:

1. GNU LESSER GENERAL PUBLIC LICENSE version 2.1
   (See file LICENSE.LGPL provided in LibRaw distribution archive for details).

2. COMMON DEVELOPMENT AND DISTRIBUTION LICENSE (CDDL) Version 1.0
   (See file LICENSE.CDDL provided in LibRaw distribution archive for details).


 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>
#include <new>
#include <string>
#include <vector>
#include <algorithm>
#include "libraw/libraw.h"

#ifdef USE_DNGSDK
#include "dng_host.h"
#endif

#ifndef LIBRAW_WIN32_CALLS
#include <netinet/in.h>
#include <dirent.h>
#include <sys/stat.h>
#else
#include <winsock2.h>
#endif

#ifdef LIBRAW_WIN32_CALLS
#define snprintf _snprintf
#define DIRSEP '\\'
#else
#define DIRSEP '/'
#endif

#define P1 RawProcessor.imgdata.idata
#define S RawProcessor.imgdata.sizes
#define C RawProcessor.imgdata.color
#define OUT RawProcessor.imgdata.params
#define OUTR RawProcessor.imgdata.rawparams

/* directory part of a path, without trailing separator (empty if none) */
static std::string dir_part(const std::string &path)
{
  size_t p = path.find_last_of("/\\");
  return p == std::string::npos ? std::string() : path.substr(0, p);
}

/* file name part of a path */
static std::string base_name(const std::string &path)
{
  size_t p = path.find_last_of("/\\");
  return p == std::string::npos ? path : path.substr(p + 1);
}

static int has_suffix(const std::string &s, const char *suffix)
{
  size_t n = strlen(suffix);
  return s.size() >= n && s.compare(s.size() - n, n, suffix) == 0;
}

/* result of a previous run: do not merge our own output files again */
static int is_own_output(const std::string &path)
{
  std::string name = base_name(path);
  for (size_t i = 0; i < name.size(); i++)
    name[i] = (char)tolower((unsigned char)name[i]);
  return has_suffix(name, ".mono.tiff") || has_suffix(name, ".mono.pgm");
}

static std::string join_path(const std::string &dir, const std::string &name)
{
  if (dir.empty())
    return name;
  std::string d = dir;
  while (d.size() > 1 && (d[d.size() - 1] == '/' || d[d.size() - 1] == '\\'))
    d.erase(d.size() - 1);
  return d + DIRSEP + name;
}

static int is_dir(const std::string &path)
{
#ifdef LIBRAW_WIN32_CALLS
  DWORD attr = GetFileAttributesA(path.c_str());
  return attr != INVALID_FILE_ATTRIBUTES && (attr & FILE_ATTRIBUTE_DIRECTORY);
#else
  struct stat st;
  return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
#endif
}

/* list regular files (no subdirectories) of a directory */
static int list_dir(const std::string &path, std::vector<std::string> &files)
{
#ifdef LIBRAW_WIN32_CALLS
  WIN32_FIND_DATAA fd;
  HANDLE h = FindFirstFileA(join_path(path, "*").c_str(), &fd);
  if (h == INVALID_HANDLE_VALUE)
    return 0;
  do
  {
    if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
      continue;
    files.push_back(join_path(path, fd.cFileName));
  } while (FindNextFileA(h, &fd));
  FindClose(h);
#else
  DIR *d = opendir(path.c_str());
  if (!d)
    return 0;
  struct dirent *e;
  while ((e = readdir(d)) != NULL)
  {
    if (!strcmp(e->d_name, ".") || !strcmp(e->d_name, ".."))
      continue;
    std::string full = join_path(path, e->d_name);
    struct stat st;
    if (stat(full.c_str(), &st) != 0 || !S_ISREG(st.st_mode))
      continue;
    files.push_back(full);
  }
  closedir(d);
#endif
  return (int)files.size();
}

static const char *exception_name(LibRaw_exceptions e)
{
  switch (e)
  {
  case LIBRAW_EXCEPTION_ALLOC:
    return "out of memory";
  case LIBRAW_EXCEPTION_DECODE_RAW:
    return "raw data decoding error";
  case LIBRAW_EXCEPTION_DECODE_JPEG:
    return "JPEG decoding error";
  case LIBRAW_EXCEPTION_DECODE_JPEG2000:
    return "JPEG2000 decoding error";
  case LIBRAW_EXCEPTION_IO_EOF:
    return "unexpected end of file";
  case LIBRAW_EXCEPTION_IO_CORRUPT:
    return "corrupt data";
  case LIBRAW_EXCEPTION_IO_BADFILE:
    return "bad file";
  case LIBRAW_EXCEPTION_TOOBIG:
    return "image too big for TIFF output";
  case LIBRAW_EXCEPTION_UNSUPPORTED_FORMAT:
    return "unsupported format";
  default:
    return "LibRaw exception";
  }
}

/* Merge one raw file. Returns 0 if the result was written, 1 if the file was
   skipped (the reason is printed in that case) */
static int merge_one(LibRaw &RawProcessor, const std::string &infile,
                     const std::string &outdir, int autoscale,
                     int black_subtraction, int use_float)
{
  int ret;

  if ((ret = RawProcessor.open_file(infile.c_str())) != LIBRAW_SUCCESS)
  {
    fprintf(stderr, "Cannot open %s: %s\n", infile.c_str(),
            libraw_strerror(ret));
    return 1; // no recycle b/c open file will recycle itself
  }
  if (P1.is_foveon)
  {
    printf("Cannot process Foveon image %s\n", infile.c_str());
    return 1;
  }
  if (P1.filters == 0)
  {
    printf("Cannot merge %s: no Bayer CFA pattern (monochrome data or "
           "already interpolated image)\n",
           infile.c_str());
    return 1;
  }
  if (P1.filters == 9)
  {
    printf("Cannot merge %s: X-Trans sensor, not a Bayer CFA\n",
           infile.c_str());
    return 1;
  }
  if ((ret = RawProcessor.unpack()) != LIBRAW_SUCCESS)
  {
    fprintf(stderr, "Cannot unpack %s: %s\n", infile.c_str(),
            libraw_strerror(ret));
    return 1;
  }
  if ((ret = RawProcessor.raw2image()) != LIBRAW_SUCCESS)
  {
    fprintf(stderr, "Cannot convert %s to image: %s\n", infile.c_str(),
            libraw_strerror(ret));
    return 1;
  }

  if (S.iwidth == S.width && S.iheight == S.height)
  {
    fprintf(stderr, "Cannot merge %s: 2x2 image mode is not available\n",
            infile.c_str());
    return 1;
  }

  printf("Black level=%d, source %ux%u -> output %ux%u\n", C.black, S.width,
         S.height, S.iwidth, S.iheight);
  if (black_subtraction)
    RawProcessor.subtract_black();

  // The half sized image holds one pixel per 2x2 CFA block, with the 4 block
  // samples in channels [0..3]. The block at (orow, ocol) is image element
  // orow*out_w+ocol, i.e. the merged value can replace the block samples in
  // place: the image buffer and its size stay untouched
  ushort(*img)[4] = RawProcessor.imgdata.image;
  unsigned out_w = S.iwidth, out_h = S.iheight;
  unsigned in_w = S.width, in_h = S.height;
  size_t out_pixels = (size_t)out_w * out_h;
  size_t idx = 0;

  for (unsigned orow = 0; orow < out_h; orow++)
  {
    // odd image dimensions leave the last block incomplete: its missing
    // samples were never filled in by raw2image(), so do not count them
    unsigned nrows = (2 * orow + 1 < in_h) ? 2 : 1;
    for (unsigned ocol = 0; ocol < out_w; ocol++, idx++)
    {
      unsigned n = nrows * ((2 * ocol + 1 < in_w) ? 2 : 1);
      ushort *px = img[idx];
      if (use_float)
      {
        double acc = 0.0;
        for (int ch = 0; ch < 4; ch++)
          acc += px[ch];
        px[0] = (ushort)floor(acc / n + 0.5);
      }
      else
      {
        uint32_t acc = 0;
        for (int ch = 0; ch < 4; ch++)
          acc += px[ch];
        px[0] = (ushort)((acc + n / 2) / n);
      }
    }
  }

  if (autoscale)
  {
    uint32_t max = 0, scale = 1;
    for (size_t j = 0; j < out_pixels; j++)
      if (max < img[j][0])
        max = img[j][0];
    if (max > 0 && max < 1 << 15)
    {
      scale = (1 << 16) / max;
      printf("Scaling with multiplier=%u (max=%u)\n", scale, max);
      for (size_t j = 0; j < out_pixels; j++)
      {
        uint32_t val = (uint32_t)img[j][0] * scale;
        img[j][0] = (ushort)(val > 0xffff ? 0xffff : val);
      }
    }
  }

  // hack to make dcraw tiff/ppm writer happy
  P1.colors = 1;
  S.width = out_w;
  S.height = out_h;
  S.iwidth = out_w;
  S.iheight = out_h;

  std::string outname = base_name(infile);
  if (OUTR.shot_select)
  {
    char sel[32];
    snprintf(sel, sizeof(sel), "-%d", OUTR.shot_select);
    outname += sel;
  }
  outname += OUT.output_tiff ? ".mono.tiff" : ".mono.pgm";
  std::string outpath = join_path(outdir, outname);

  printf("Writing file %s\n", outpath.c_str());
  if (LIBRAW_SUCCESS != (ret = RawProcessor.dcraw_ppm_tiff_writer(outpath.c_str())))
  {
    fprintf(stderr, "Cannot write %s: %s\n", outpath.c_str(),
            libraw_strerror(ret));
    return 1;
  }
  return 0;
}

int main(int ac, char *av[])
{
  int i;
  int autoscale = 0, black_subtraction = 1, use_gamma = 0, use_pgm = 0,
      use_float = 0;
  const char *indir = NULL, *outdir = NULL;
  std::vector<std::string> files;

  LibRaw RawProcessor;
#ifdef USE_DNGSDK
  dng_host *dnghost = nullptr;
#endif

  // unbuffered stdout: with a buffered stdout the progress message of file N+1
  // may show up before the work on file N is done, which looks like the files
  // are processed in parallel. They are not: this program handles one file at
  // a time, from open to write.
  setvbuf(stdout, NULL, _IONBF, 0);

  if (ac < 2)
  {
  usage:
    printf("bayer_merge - LibRaw %s sample. %d cameras supported\n"
           "Usage: %s [options] raw-files....\n"
           "       %s [options] --indir DIRECTORY\n"
           "\tAverages each Bayer 2x2 CFA block into one monochrome pixel;\n"
           "\tthe output holds 1/4 of the source pixels in 16-bit precision\n"
           "\t--indir DIR  - merge all files of DIR (subdirectories are not "
           "scanned)\n"
           "\t--outdir DIR - write output to DIR; default: the directory of "
           "the source file\n"
           "\t-s N - select Nth image in file (default=0)\n"
           "\t-g - use gamma correction with gamma 2.2 (not precise,use for "
           "visual inspection only)\n"
           "\t-A - autoscaling (by integer factor)\n"
           "\t-B - no black subtraction\n"
           "\t-p - write 16-bit PGM instead of 16-bit TIFF\n"
           "\t-f - average in double precision instead of 32-bit unsigned "
           "integers\n",
           LibRaw::version(), LibRaw::cameraCount(), av[0], av[0]);
    return 1;
  }

  for (i = 1; i < ac; i++)
  {
    if (av[i][0] == '-' && av[i][1] == '-')
    {
      if (!strcmp(av[i], "--indir"))
      {
        if (++i == ac)
          goto usage;
        indir = av[i];
      }
      else if (!strcmp(av[i], "--outdir"))
      {
        if (++i == ac)
          goto usage;
        outdir = av[i];
      }
      else
        goto usage;
    }
    else if (av[i][0] == '-' && av[i][1])
    {
      if (av[i][1] == 's' && av[i][2] == 0)
      {
        if (++i == ac)
          goto usage;
        OUTR.shot_select = av[i] ? atoi(av[i]) : 0;
      }
      else if (av[i][1] == 'g' && av[i][2] == 0)
        use_gamma = 1;
      else if (av[i][1] == 'A' && av[i][2] == 0)
        autoscale = 1;
      else if (av[i][1] == 'B' && av[i][2] == 0)
        black_subtraction = 0;
      else if (av[i][1] == 'p' && av[i][2] == 0)
        use_pgm = 1;
      else if (av[i][1] == 'f' && av[i][2] == 0)
        use_float = 1;
      else
        goto usage;
    }
    else
      files.push_back(av[i]);
  }

  if (indir)
  {
    if (!is_dir(indir))
    {
      fprintf(stderr, "Input directory %s not found\n", indir);
      return 1;
    }
    std::vector<std::string> dirfiles;
    list_dir(indir, dirfiles);
    std::sort(dirfiles.begin(), dirfiles.end());
    unsigned n_own = 0;
    for (size_t k = 0; k < dirfiles.size(); k++)
    {
      if (is_own_output(dirfiles[k]))
      {
        n_own++;
        continue;
      }
      files.push_back(dirfiles[k]);
    }
    printf("%u file(s) found in %s", (unsigned)(dirfiles.size() - n_own), indir);
    if (n_own)
      printf(" (%u result file(s) of a previous run ignored)", n_own);
    printf("\n");
  }
  if (files.empty())
    goto usage;
  if (outdir && !is_dir(outdir))
  {
    fprintf(stderr, "Output directory %s not found\n", outdir);
    return 1;
  }

  OUT.output_bps = 16;
  OUT.output_tiff = !use_pgm;
  OUT.user_flip = 0;
  OUT.no_auto_bright = 1;
  // half_size makes raw2image() collect the CFA grid into the 4 channels of
  // a half sized image, one channel per Bayer color of the 2x2 block
  OUT.half_size = 1;
  // linear output, unless gamma correction was requested
  if (!use_gamma)
    OUT.gamm[0] = OUT.gamm[1] = 1;

#ifdef USE_DNGSDK
  // Set up DNG host for DNG SDK support (required for JXL DNG files)
  dnghost = new dng_host;
  RawProcessor.set_dng_host(dnghost);
#endif

  int n_merged = 0, n_skipped = 0;

  // strictly sequential: one file is opened, unpacked, merged and written
  // before the next one is started
  for (size_t fi = 0; fi < files.size(); fi++)
  {
    const char *fname = files[fi].c_str();
    printf("[%u/%u] Processing file %s\n", (unsigned)(fi + 1),
           (unsigned)files.size(), fname);

    int ok = 0;
    try
    {
      ok = merge_one(RawProcessor, files[fi],
                     outdir ? std::string(outdir) : dir_part(files[fi]),
                     autoscale, black_subtraction, use_float) == 0;
    }
    catch (const LibRaw_exceptions &e)
    {
      fprintf(stderr, "Cannot process %s: %s (%d)\n", fname, exception_name(e),
              (int)e);
    }
    catch (const std::bad_alloc &)
    {
      fprintf(stderr, "Cannot process %s: out of memory\n", fname);
    }
    catch (...)
    {
      fprintf(stderr, "Cannot process %s: unexpected error\n", fname);
    }

    if (ok)
      n_merged++;
    else
      n_skipped++;

    // drop all per-file state before the next file. open_file() recycles too,
    // but it does not do so when the file cannot be opened at all
    RawProcessor.recycle();
  }

  printf("Done: %d file(s) merged, %d skipped\n", n_merged, n_skipped);
  return n_merged ? 0 : 1;
}
