// Copyright 2016 Preferred Networks, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "file_funcs.h"


bool save_time_fname(pcl::PointCloud<pcl::PointXYZRGB>& cld)
{
	time_t timer;
	time(&timer);
	struct tm *t_st = localtime(&timer);

	char fname[100];
	sprintf(fname, "%4d%02d%02d_%02d%02d%02d.pcd",
		t_st->tm_year + 1900,
		t_st->tm_mon + 1,
		t_st->tm_mday,
		t_st->tm_hour,
		t_st->tm_min,
		t_st->tm_sec);

	printf("save file %s\n", fname);

	int ret = pcl::io::savePCDFileASCII(fname, cld);

	
	return true;
}