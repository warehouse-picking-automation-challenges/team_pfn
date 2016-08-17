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

#include "load_item_list.h"

bool LoadItemList(const char* fname, item_list& l)
{
	FILE* fp = fopen(fname,"r");
	if(fp == NULL)
		return false;
	
	char id[10];
	char name[100];
	char desc[100];
	char count[10];
	char bonus[100];
	int n = fscanf(fp,"%[^,],%[^,],%[^,],%[^,],%s\n",id, name, desc, count, bonus);
	if( n != 5 ){ fclose(fp); return false; }	
	
	l.list.clear();
	l.name_to_id.clear();
	
	for(int i=0;i<39;i++)
	{
		int n = fscanf(fp,"%[^,],%[^,],%[^,],%[^,],%s\n",id, name, desc, count, bonus);
		if( n != 5 ){ fclose(fp); return false; }	
		
		if(atoi(id) != i+1){ fclose(fp); return false; }
		
		item_elem itm;
		itm.id = atoi(id);
		itm.name = name;
		itm.description = desc;
		itm.count = atoi(count);
		itm.bonus = atoi(bonus);
		
		l.list.push_back(itm);
		l.name_to_id[name] = atoi(id);
	}
	
	return true;
}