/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Open Source Robotics Foundation, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/transport/transport.h"
#include "ros/console.h"

#include <winsock2.h>
#include <iphlpapi.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "iphlpapi.lib")

#include<vector>
#include<string>

//#include <netinet/in.h>
//#include <sys/socket.h>
//#include <netdb.h>
//
//#if !defined(__ANDROID__)
//#include <ifaddrs.h>
//#endif

#ifndef NI_MAXHOST
  #define NI_MAXHOST 1025
#endif


void EnumMyAddress(std::vector<std::string>& addrlist)
{
	ULONG len = 0;   // 列挙に必要なバイト数です。
	DWORD ret = GetAdaptersAddresses(AF_UNSPEC, 0, 0, 0, &len);
	if (ret != ERROR_BUFFER_OVERFLOW) return;   // メモリ不足以外のエラーなら制御を返します。

												// 要求されたバイト数 len 分のメモリを adpts に用意します。
	PIP_ADAPTER_ADDRESSES adpts = (PIP_ADAPTER_ADDRESSES)new BYTE[len];
	if (adpts == 0) return;      // メモリを用意できなければ制御を返します。

	ret = GetAdaptersAddresses(AF_UNSPEC, 0, 0, adpts, &len);   // adpts に情報を列挙します。
	if (ret != ERROR_SUCCESS) {   // アダプタを列挙できなかったら制御を返します。
		delete[] adpts;
		return;
	}

	WSADATA data;
	ZeroMemory(&data, sizeof(data));
	WSAStartup(MAKEWORD(2, 2), &data);
	// adpts 配列内の各アダプタ情報を一つずつ adpt に入れてみていきます。
	for (PIP_ADAPTER_ADDRESSES adpt = adpts; adpt; adpt = adpt->Next) {
		if (adpt->PhysicalAddressLength == 0) continue;            // データリンク層を持たないなら
		if (adpt->IfType == IF_TYPE_SOFTWARE_LOOPBACK) continue;   // ループバック アドレスなら
																   // adpt アダプタ情報の IP アドレスを一つずつ uni に入れてみていきます。
		for (PIP_ADAPTER_UNICAST_ADDRESS uni = adpt->FirstUnicastAddress; uni; uni = uni->Next) {
			// DNS に対して適切なアドレスでない（プライベートなど）ならば
			if (~(uni->Flags) & IP_ADAPTER_ADDRESS_DNS_ELIGIBLE) continue;   // 次のアドレスへ
																			 // アプリケーションが使うべきでないアドレスなら
			if (uni->Flags & IP_ADAPTER_ADDRESS_TRANSIENT) continue;         // 次のアドレスへ
			char host[NI_MAXHOST + 1] = { '\0' };   // host に「0.0.0.0」形式の文字列を取得します。
			if (getnameinfo(uni->Address.lpSockaddr, uni->Address.iSockaddrLength
				, host, sizeof(host), 0, 0, NI_NUMERICHOST/*NI_NAMEREQD*/) == 0)
			{
				 // 取得した「0.0.0.0」表記をここで処理
				addrlist.push_back(std::string(host));
			}
		}
	}
	WSACleanup();
	delete[] adpts;
}



namespace ros
{

Transport::Transport()
: only_localhost_allowed_(false)
{
  char *ros_ip_env = NULL, *ros_hostname_env = NULL;
  #ifdef _MSC_VER
    _dupenv_s(&ros_ip_env, NULL, "ROS_IP");
    _dupenv_s(&ros_hostname_env, NULL, "ROS_HOSTNAME");
  #else
    ros_ip_env = getenv("ROS_IP");
    ros_hostname_env = getenv("ROS_HOSTNAME");
  #endif

  if (ros_hostname_env && !strcmp(ros_hostname_env, "localhost"))
    only_localhost_allowed_ = true;
  else if (ros_ip_env && !strncmp(ros_ip_env, "127.", 4))
    only_localhost_allowed_ = true;
  else if (ros_ip_env && !strcmp(ros_ip_env, "::1"))
    only_localhost_allowed_ = true;

  char our_hostname[256] = {0};
  gethostname(our_hostname, sizeof(our_hostname)-1);
  allowed_hosts_.push_back(std::string(our_hostname));
  allowed_hosts_.push_back("localhost");

  /*
#if !defined(__ANDROID__)
  // for ipv4 loopback, we'll explicitly search for 127.* in isHostAllowed()
  // now we need to iterate all local interfaces and add their addresses
  // from the getifaddrs manpage:  (maybe something similar for windows ?) 
  ifaddrs *ifaddr;
  if (-1 == getifaddrs(&ifaddr))
  {
    ROS_ERROR("getifaddr() failed");
    return;
  }
  for (ifaddrs *ifa = ifaddr; ifa; ifa = ifa->ifa_next)
  {
    if(NULL == ifa->ifa_addr)
      continue; // ifa_addr can be NULL

    int family = ifa->ifa_addr->sa_family;
    if (family != AF_INET && family != AF_INET6)
      continue; // we're only looking for IP addresses

    char addr[NI_MAXHOST] = {0};
    if (getnameinfo(ifa->ifa_addr,
                    (family == AF_INET) ? sizeof(sockaddr_in)
                                        : sizeof(sockaddr_in6),
                    addr, NI_MAXHOST,
                    NULL, 0, NI_NUMERICHOST))
    {
      ROS_ERROR("getnameinfo() failed");
      continue;
    }
    allowed_hosts_.push_back(std::string(addr));
  }
  freeifaddrs(ifaddr);
#endif

  */


  std::vector<std::string>	list;
  EnumMyAddress(list);

  for (int i = 0; i < list.size(); i++)
  {
	  allowed_hosts_.push_back(list[i]);
  }


}

bool Transport::isHostAllowed(const std::string &host) const
{
  if (!only_localhost_allowed_)
    return true; // doesn't matter; we'll connect to anybody

  if (host.length() >= 4 && host.substr(0, 4) == std::string("127."))
    return true; // ipv4 localhost
  // now, loop through the list of valid hostnames and see if we find it
  for (std::vector<std::string>::const_iterator it = allowed_hosts_.begin(); 
       it != allowed_hosts_.end(); ++it)
  {
    if (host == *it)
      return true; // hooray
  }
  ROS_WARN("ROS_HOSTNAME / ROS_IP is set to only allow local connections, so "
           "a requested connection to '%s' is being rejected.", host.c_str());
  return false; // sadness
}

}

