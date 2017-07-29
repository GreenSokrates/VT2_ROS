/*
 * HTTPServer.cpp
 * Copyright (c) 2014, ZHAW
 * All rights reserved.
 *
 *  Created on: 11.12.2014
 *      Author: Marcel Honegger
 */

#include <http_server/HTTPScript.h>
#include <http_server/HTTPServer.h>

using namespace std;

template <class T>
inline string type2String(const T &t)
{

	stringstream out;
	out << fixed << setprecision(3) << t;

	return out.str();
}

/**
 * Creates an http server with the default port number.
 */
HTTPServer::HTTPServer() : Thread("HTTPServer", STACK_SIZE)
{

	portNumber = PORT_NUMBER;
	path = "";
	indexPage = "index.html";

	signal(SIGPIPE, SIG_IGN);
}

/**
 * Creates an http server running on a given port.
 * @param portNumber the port this server is running on.
 */
HTTPServer::HTTPServer(uint16_t portNumber) : Thread("HTTPServer", STACK_SIZE)
{

	this->portNumber = portNumber;
	this->path = "";
	this->indexPage = "index.html";

	signal(SIGPIPE, SIG_IGN);
}

/**
 * Creates an http server running on a given port, with given
 * path and name of the default html page.
 * @param portNumber the port this server is running on.
 * @param path the path name to files this server is supposed to serve.
 * @param indexPage the name of the default html page, i.e. 'index.html'.
 */
HTTPServer::HTTPServer(uint16_t portNumber, string path, string indexPage) : Thread("HTTPServer", STACK_SIZE)
{

	this->portNumber = portNumber;
	this->path = path;
	this->indexPage = indexPage;

	signal(SIGPIPE, SIG_IGN);
}

HTTPServer::~HTTPServer() {}

/**
 * Decodes a given URL string into a standard text string.
 */
string HTTPServer::urlDecoder(string url)
{

	size_t pos = -1;
	while ((pos = url.find("+")) != string::npos)
		url = url.substr(0, pos) + " " + url.substr(pos + 1);
	while ((pos = url.find("%08")) != string::npos)
		url = url.substr(0, pos) + "\b" + url.substr(pos + 3);
	while ((pos = url.find("%09")) != string::npos)
		url = url.substr(0, pos) + "\t" + url.substr(pos + 3);
	while ((pos = url.find("%0A")) != string::npos)
		url = url.substr(0, pos) + "\n" + url.substr(pos + 3);
	while ((pos = url.find("%0D")) != string::npos)
		url = url.substr(0, pos) + "\r" + url.substr(pos + 3);
	while ((pos = url.find("%20")) != string::npos)
		url = url.substr(0, pos) + " " + url.substr(pos + 3);
	while ((pos = url.find("%22")) != string::npos)
		url = url.substr(0, pos) + "\"" + url.substr(pos + 3);
	while ((pos = url.find("%23")) != string::npos)
		url = url.substr(0, pos) + "#" + url.substr(pos + 3);
	while ((pos = url.find("%24")) != string::npos)
		url = url.substr(0, pos) + "$" + url.substr(pos + 3);
	while ((pos = url.find("%25")) != string::npos)
		url = url.substr(0, pos) + "%" + url.substr(pos + 3);
	while ((pos = url.find("%26")) != string::npos)
		url = url.substr(0, pos) + "&" + url.substr(pos + 3);
	while ((pos = url.find("%2B")) != string::npos)
		url = url.substr(0, pos) + "+" + url.substr(pos + 3);
	while ((pos = url.find("%2C")) != string::npos)
		url = url.substr(0, pos) + "," + url.substr(pos + 3);
	while ((pos = url.find("%2F")) != string::npos)
		url = url.substr(0, pos) + "/" + url.substr(pos + 3);
	while ((pos = url.find("%3A")) != string::npos)
		url = url.substr(0, pos) + ":" + url.substr(pos + 3);
	while ((pos = url.find("%3B")) != string::npos)
		url = url.substr(0, pos) + ";" + url.substr(pos + 3);
	while ((pos = url.find("%3C")) != string::npos)
		url = url.substr(0, pos) + "<" + url.substr(pos + 3);
	while ((pos = url.find("%3D")) != string::npos)
		url = url.substr(0, pos) + "=" + url.substr(pos + 3);
	while ((pos = url.find("%3E")) != string::npos)
		url = url.substr(0, pos) + ">" + url.substr(pos + 3);
	while ((pos = url.find("%3F")) != string::npos)
		url = url.substr(0, pos) + "?" + url.substr(pos + 3);
	while ((pos = url.find("%40")) != string::npos)
		url = url.substr(0, pos) + "@" + url.substr(pos + 3);

	return url;
}

/**
 * Registers the given script with the http server.
 * This allows to call a method of this script object
 * through virtual cgi-bin requests from a remote system.
 */
void HTTPServer::add(string name, HTTPScript *httpScript)
{

	httpScriptNames.push_back(name);
	httpScripts.push_back(httpScript);
}

void HTTPServer::run()
{

	int32_t serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	sockaddr_in serverAddr;

	memset(&serverAddr, 0, sizeof(serverAddr));
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddr.sin_port = htons(portNumber);

	while (true)
	{

		if (bind(serverSocket, (sockaddr *)&serverAddr, sizeof(serverAddr)) == 0)
		{

			listen(serverSocket, 5);

			while (true)
			{

				sockaddr_in clientAddr;
				socklen_t clientLength = sizeof(clientAddr);

				int32_t clientSocket = accept(serverSocket, (sockaddr *)&clientAddr, &clientLength);
				if (clientSocket != -1)
				{

					// create client thread to handle request

					HTTPClientThread *httpClientThread = new HTTPClientThread(this, clientSocket);
					httpClientThread->start();
					httpClients.push_back(httpClientThread);

					// clean up dead client threads

					for (list<HTTPClientThread *>::iterator i = httpClients.begin(); i != httpClients.end();)
					{
						HTTPClientThread *httpClientThread = *i;
						if (httpClientThread->isAlive())
						{
							++i;
						}
						else
						{
							httpClientThread->join();
							i = httpClients.erase(i);
							delete httpClientThread;
						}
					}
				}
			}
		}

		sleep(BIND_RETRY_DELAY);
	}

	close(serverSocket);
}

/**
 * Creates an http client thread.
 */
HTTPClientThread::HTTPClientThread(HTTPServer *httpServer, int32_t clientSocket) : Thread("HTTPServer", HTTPServer::STACK_SIZE)
{

	this->httpServer = httpServer;
	this->clientSocket = clientSocket;
}

HTTPClientThread::~HTTPClientThread() {}

void HTTPClientThread::run()
{

	try
	{

		timeval tv;
		tv.tv_sec = 60;
		tv.tv_usec = 0;
		setsockopt(clientSocket, SOL_SOCKET, SO_RCVTIMEO, (void *)&tv, sizeof(tv));

		char buffer[HTTPServer::BUFFER_SIZE];
		bzero(buffer, HTTPServer::BUFFER_SIZE);

		uint32_t size = read(clientSocket, buffer, HTTPServer::BUFFER_SIZE - 1);
		while (size > 0)
		{

			string input(buffer, size);
			string header;
			string output;

			// check if input is complete

			if (input.find("\r\n\r\n") == string::npos)
			{
				sleep(HTTPServer::READ_RETRY_DELAY);
				if ((size = read(clientSocket, buffer, HTTPServer::BUFFER_SIZE - 1)) > 0)
					input += string(buffer, size);
			}

			// parse input

			if ((input.find("GET") == 0) || (input.find("HEAD") == 0))
			{

				if (input.find("cgi-bin") != string::npos)
				{

					// process script request with arguments

					string script = input.substr(input.find("cgi-bin/") + 8, input.find(" ", input.find("cgi-bin/") + 8) - input.find("cgi-bin/") - 8);
					string name;
					vector<string> names;
					vector<string> values;

					if (script.find("?") != string::npos)
					{

						name = script.substr(0, script.find("?"));
						script = script.substr(script.find("?") + 1);

						vector<string> arguments;
						while (script.find("&") != string::npos)
						{
							arguments.push_back(script.substr(0, script.find("&")));
							script = script.substr(script.find("&") + 1);
						}
						arguments.push_back(script);

						for (uint32_t i = 0; i < arguments.size(); i++)
						{

							if (arguments[i].find("=") != string::npos)
							{

								names.push_back(arguments[i].substr(0, arguments[i].find("=")));
								values.push_back(httpServer->urlDecoder(arguments[i].substr(arguments[i].find("=") + 1)));
							}
							else
							{

								names.push_back(arguments[i]);
								values.push_back("");
							}
						}
					}
					else
					{

						name = script;
					}

					// look for corresponding script

					for (uint32_t i = 0; i < min(httpServer->httpScriptNames.size(), httpServer->httpScripts.size()); i++)
					{

						if (httpServer->httpScriptNames[i].compare(name) == 0)
						{

							output = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n";
							output += "<!DOCTYPE html>\r\n";
							output += "<html xmlns=\"http://www.w3.org/1999/xhtml\" xml:lang=\"en\" lang=\"en\">\r\n";
							output += "<body>\r\n";
							output += httpServer->httpScripts[i]->call(names, values);
							output += "</body>\r\n";
							output += "</html>\r\n";

							header = "HTTP/1.1 200 OK\r\n";
							header += "Content-Length: " + type2String(output.size()) + "\r\n";
							header += "Content-Type: text/xml\r\n";
							header += "Expires: 0\r\n";
							header += "\r\n";

							output = header + output;
						}
					}

					// requested script was not found on this server

					if (output.size() == 0)
					{

						output += "<!DOCTYPE html>\r\n";
						output += "<html lang=\"en\">\r\n";
						output += "<head>\r\n";
						output += "  <title>400 Bad Request</title>\r\n";
						output += "  <style type=\"text/css\">\r\n";
						output += "    h2 {font-family:Helvetica,Arial,sans-serif; font-size: 24; color:#FFFFFF;}\r\n";
						output += "    p {font-family:Helvetica,Arial,sans-serif; font-size: 14; color:#003253;}\r\n";
						output += "  </style>\r\n";
						output += "</head>\r\n";
						output += "<body leftmargin=\"0\" topmargin=\"0\" marginwidth=\"0\" marginheight=\"0\">\r\n";
						output += "  <table width=\"100%\" height=\"100%\" border=\"0\" frame=\"void\" cellspacing=\"0\" cellpadding=\"20\">\r\n";
						output += "    <tr>\r\n";
						output += "      <td width=\"100%\" height=\"30\" bgcolor=\"#0064A6\"><h2>400 Bad Request</h2></td>\r\n";
						output += "    </tr>\r\n";
						output += "    <tr>\r\n";
						output += "      <td valign=\"top\" style=\"border-top-width:2px; border-left-width:0px; border-bottom-width:0px; border-right-width:0px; border-style:solid; border-color:#003253;\">\r\n";
						output += "      <p>The requested script could not be found on this server!</p>\r\n";
						output += "      </td>\r\n";
						output += "    </tr>\r\n";
						output += "  </table>\r\n";
						output += "</body>\r\n";
						output += "</html>\r\n";

						header = "HTTP/1.1 400 Bad Request\r\n";
						header += "Content-Length: " + type2String(output.size()) + "\r\n";
						header += "Content-Type: text/html\r\n";
						header += "\r\n";

						output = header + output;
					}

					ssize_t written = write(clientSocket, output.c_str(), output.size());
					if (written != static_cast<ssize_t>(output.size()))
						throw runtime_error("HTTPServer: couldn't write content to socket.");
				}
				else
				{

					// look for file to load and transmit

					string filename = input.substr(input.find("/") + 1, input.find(" ", input.find("/")) - input.find("/") - 1);
					if (filename.size() == 0)
						filename = httpServer->indexPage;
					filename = httpServer->path + filename;

					ifstream file;
					file.open(filename.c_str());
					if (file)
					{

						// requested file exists

						file.seekg(0, ios::end);
						int32_t n = file.tellg();

						output = "HTTP/1.1 200 OK\r\n";
						output += "Content-Length: " + type2String(n) + "\r\n";

						if (filename.find(".html") != string::npos)
							output += "Content-Type: text/html\r\n";
						else if (filename.find(".htm") != string::npos)
							output += "Content-Type: text/html\r\n";
						else if (filename.find(".text") != string::npos)
							output += "Content-Type: text/plain\r\n";
						else if (filename.find(".txt") != string::npos)
							output += "Content-Type: text/plain\r\n";
						else if (filename.find(".conf") != string::npos)
							output += "Content-Type: text/plain\r\n";
						else if (filename.find(".asc") != string::npos)
							output += "Content-Type: text/plain\r\n";
						else if (filename.find(".css") != string::npos)
							output += "Content-Type: text/css\r\n";
						else if (filename.find(".class") != string::npos)
							output += "Content-Type: application/octet-stream\r\n";
						else if (filename.find(".c") != string::npos)
							output += "Content-Type: text/plain\r\n";
						else if (filename.find(".xml") != string::npos)
							output += "Content-Type: text/xml\r\n";
						else if (filename.find(".dtd") != string::npos)
							output += "Content-Type: text/xml\r\n";
						else if (filename.find(".js") != string::npos)
							output += "Content-Type: text/javascript\r\n";
						else if (filename.find(".gif") != string::npos)
							output += "Content-Type: image/gif\r\n";
						else if (filename.find(".jpg") != string::npos)
							output += "Content-Type: image/jpeg\r\n";
						else if (filename.find(".jpeg") != string::npos)
							output += "Content-Type: image/jpeg\r\n";
						else if (filename.find(".png") != string::npos)
							output += "Content-Type: image/png\r\n";
						else if (filename.find(".xbm") != string::npos)
							output += "Content-Type: image/x-xbitmap\r\n";
						else if (filename.find(".xpm") != string::npos)
							output += "Content-Type: image/x-xpixmap\r\n";
						else if (filename.find(".xwd") != string::npos)
							output += "Content-Type: image/x-xwindowdump\r\n";
						else if (filename.find(".manifest") != string::npos)
							output += "Content-Type: text/cache-manifest\r\n";
						else if (filename.find(".jar") != string::npos)
							output += "Content-Type: application/x-java-applet\r\n";
						else if (filename.find(".pdf") != string::npos)
							output += "Content-Type: application/pdf\r\n";
						else if (filename.find(".sig") != string::npos)
							output += "Content-Type: application/pgp-signature\r\n";
						else if (filename.find(".spl") != string::npos)
							output += "Content-Type: application/futuresplash\r\n";
						else if (filename.find(".ps") != string::npos)
							output += "Content-Type: application/postscript\r\n";
						else if (filename.find(".torrent") != string::npos)
							output += "Content-Type: application/x-bittorrent\r\n";
						else if (filename.find(".dvi") != string::npos)
							output += "Content-Type: application/x-dvi\r\n";
						else if (filename.find(".pac") != string::npos)
							output += "Content-Type: application/x-ns-proxy-autoconfig\r\n";
						else if (filename.find(".swf") != string::npos)
							output += "Content-Type: application/x-shockwave-flash\r\n";
						else if (filename.find(".tar.gz") != string::npos)
							output += "Content-Type: application/x-tgz\r\n";
						else if (filename.find(".tar.bz2") != string::npos)
							output += "Content-Type: application/x-bzip-compressed-tar\r\n";
						else if (filename.find(".gz") != string::npos)
							output += "Content-Type: application/x-gzip\r\n";
						else if (filename.find(".tgz") != string::npos)
							output += "Content-Type: application/x-tgz\r\n";
						else if (filename.find(".tar") != string::npos)
							output += "Content-Type: application/x-tar\r\n";
						else if (filename.find(".bz2") != string::npos)
							output += "Content-Type: application/x-bzip\r\n";
						else if (filename.find(".tbz") != string::npos)
							output += "Content-Type: application/x-bzip-compressed-tar\r\n";
						else if (filename.find(".zip") != string::npos)
							output += "Content-Type: application/zip\r\n";
						else if (filename.find(".mp3") != string::npos)
							output += "Content-Type: audio/mpeg\r\n";
						else if (filename.find(".m3u") != string::npos)
							output += "Content-Type: audio/x-mpegurl\r\n";
						else if (filename.find(".wma") != string::npos)
							output += "Content-Type: audio/x-ms-wma\r\n";
						else if (filename.find(".wax") != string::npos)
							output += "Content-Type: audio/x-ms-wax\r\n";
						else if (filename.find(".wav") != string::npos)
							output += "Content-Type: audio/x-wav\r\n";
						else if (filename.find(".ogg") != string::npos)
							output += "Content-Type: audio/x-wav\r\n";
						else if (filename.find(".mpeg") != string::npos)
							output += "Content-Type: video/mpeg\r\n";
						else if (filename.find(".mpg") != string::npos)
							output += "Content-Type: video/mpeg\r\n";
						else if (filename.find(".mp4") != string::npos)
							output += "Content-Type: video/mp4\r\n";
						else if (filename.find(".mov") != string::npos)
							output += "Content-Type: video/quicktime\r\n";
						else if (filename.find(".qt") != string::npos)
							output += "Content-Type: video/quicktime\r\n";
						else if (filename.find(".ogv") != string::npos)
							output += "Content-Type: video/ogg\r\n";
						else if (filename.find(".avi") != string::npos)
							output += "Content-Type: video/x-msvideo\r\n";
						else if (filename.find(".asf") != string::npos)
							output += "Content-Type: video/x-ms-asf\r\n";
						else if (filename.find(".asx") != string::npos)
							output += "Content-Type: video/x-ms-asf\r\n";
						else if (filename.find(".wmv") != string::npos)
							output += "Content-Type: video/x-ms-wmv\r\n";
						else if (filename.find(".webm") != string::npos)
							output += "Content-Type: video/webm\r\n";

						output += "\r\n";

						ssize_t written = write(clientSocket, output.c_str(), output.size());
						if (written != static_cast<ssize_t>(output.size()))
							throw runtime_error("HTTPServer: couldn't write content to socket.");

						if (input.find("GET") == 0)
						{

							// transmit file

							file.seekg(0, ios::beg);
							if (file)
							{
								char fileBuffer[1024];
								for (int32_t i = 0; i < n / 1024; i++)
								{
									file.read(fileBuffer, 1024);
									written = write(clientSocket, fileBuffer, 1024);
									if (written != 1024)
										throw runtime_error("couldn't write content to socket.");
								}
								file.read(fileBuffer, n % 1024);
								written = write(clientSocket, fileBuffer, n % 1024);
								if (written != n % 1024)
									throw runtime_error("couldn't write content to socket.");
							}
						}

						file.close();
					}
					else
					{

						// file not found

						output += "<!DOCTYPE html>\r\n";
						output += "<html lang=\"en\">\r\n";
						output += "<head>\r\n";
						output += "  <title>404 Not Found</title>\r\n";
						output += "  <style type=\"text/css\">\r\n";
						output += "    h2 {font-family:Helvetica,Arial,sans-serif; font-size: 24; color:#FFFFFF;}\r\n";
						output += "    p {font-family:Helvetica,Arial,sans-serif; font-size: 14; color:#003253;}\r\n";
						output += "  </style>\r\n";
						output += "</head>\r\n";
						output += "<body leftmargin=\"0\" topmargin=\"0\" marginwidth=\"0\" marginheight=\"0\">\r\n";
						output += "  <table width=\"100%\" height=\"100%\" border=\"0\" frame=\"void\" cellspacing=\"0\" cellpadding=\"20\">\r\n";
						output += "    <tr>\r\n";
						output += "      <td width=\"100%\" height=\"30\" bgcolor=\"#0064A6\"><h2>404 Not Found</h2></td>\r\n";
						output += "    </tr>\r\n";
						output += "    <tr>\r\n";
						output += "      <td valign=\"top\" style=\"border-top-width:2px; border-left-width:0px; border-bottom-width:0px; border-right-width:0px; border-style:solid; border-color:#003253;\">\r\n";
						output += "      <p>The requested file could not be found on this server!</p>\r\n";
						output += "      </td>\r\n";
						output += "    </tr>\r\n";
						output += "  </table>\r\n";
						output += "</body>\r\n";
						output += "</html>\r\n";

						header = "HTTP/1.1 404 Not Found\r\n";
						header += "Content-Length: " + type2String(output.size()) + "\r\n";
						header += "Content-Type: text/html\r\n";
						header += "\r\n";

						output = header + output;

						ssize_t written = write(clientSocket, output.c_str(), output.size());
						if (written != static_cast<ssize_t>(output.size()))
							throw runtime_error("HTTPServer: couldn't write content to socket.");
					}
				}
			}
			else if (input.find("POST") == 0)
			{

				string filename = input.substr(input.find("/") + 1, input.find(" ", input.find("/")) - input.find("/") - 1);
				if (filename.size() > 0)
				{

					filename = httpServer->path + filename;

					char sequence[5];
					sequence[0] = (char)13;
					sequence[1] = (char)10;
					sequence[2] = (char)13;
					sequence[3] = (char)10;
					sequence[4] = (char)0;

					size_t pos = string::npos;
					if ((pos = input.find(sequence)) != string::npos)
					{

						if (pos < input.size() - 4)
						{

							input = input.substr(pos + 4);
							if ((pos = input.find("=")) != string::npos)
							{
								if (pos < input.size() - 1)
									input = input.substr(pos + 1);
							}

							// posted content is saved to a file

							ofstream posting;
							posting.open(filename.c_str());
							posting << httpServer->urlDecoder(input);
							posting.close();

							output = "HTTP/1.1 201 Created\r\n";
							output += "Content-Length: 0\r\n";
							output += "\r\n";
						}
						else
						{

							// posted content cannot be saved to a file

							output = "HTTP/1.1 403 Forbidden\r\n";
							output += "Content-Length: 0\r\n";
							output += "\r\n";
						}
					}
					else
					{

						// posted content cannot be saved to a file

						output = "HTTP/1.1 403 Forbidden\r\n";
						output += "Content-Length: 0\r\n";
						output += "\r\n";
					}

					ssize_t written = write(clientSocket, output.c_str(), output.size());
					if (written != static_cast<ssize_t>(output.size()))
						throw runtime_error("HTTPServer: couldn't write content to socket.");
				}
				else
				{

					// posted content cannot be saved to a file

					output = "HTTP/1.1 403 Forbidden\r\n";
					output += "Content-Length: 0\r\n";
					output += "\r\n";

					ssize_t written = write(clientSocket, output.c_str(), output.size());
					if (written != static_cast<ssize_t>(output.size()))
						throw runtime_error("HTTPServer: couldn't write content to socket.");
				}
			}
			else
			{

				// the http method is not known

				output += "<!DOCTYPE html>\r\n";
				output += "<html lang=\"en\">\r\n";
				output += "<head>\r\n";
				output += "  <title>400 Bad Request</title>\r\n";
				output += "  <style type=\"text/css\">\r\n";
				output += "    h2 {font-family:Helvetica,Arial,sans-serif; font-size: 24; color:#FFFFFF;}\r\n";
				output += "    p {font-family:Helvetica,Arial,sans-serif; font-size: 14; color:#003253;}\r\n";
				output += "  </style>\r\n";
				output += "</head>\r\n";
				output += "<body leftmargin=\"0\" topmargin=\"0\" marginwidth=\"0\" marginheight=\"0\">\r\n";
				output += "  <table width=\"100%\" height=\"100%\" border=\"0\" frame=\"void\" cellspacing=\"0\" cellpadding=\"20\">\r\n";
				output += "    <tr>\r\n";
				output += "      <td width=\"100%\" height=\"30\" bgcolor=\"#0064A6\"><h2>400 Bad Request</h2></td>\r\n";
				output += "    </tr>\r\n";
				output += "    <tr>\r\n";
				output += "      <td valign=\"top\" style=\"border-top-width:2px; border-left-width:0px; border-bottom-width:0px; border-right-width:0px; border-style:solid; border-color:#003253;\">\r\n";
				output += "      <p>The requested method is not supported by this server!</p>\r\n";
				output += "      </td>\r\n";
				output += "    </tr>\r\n";
				output += "  </table>\r\n";
				output += "</body>\r\n";
				output += "</html>\r\n";

				header = "HTTP/1.1 400 Bad Request\r\n";
				header += "Content-Length: " + type2String(output.size()) + "\r\n";
				header += "Content-Type: text/html\r\n";
				header += "\r\n";

				output = header + output;

				ssize_t written = write(clientSocket, output.c_str(), output.size());
				if (written != static_cast<ssize_t>(output.size()))
					throw runtime_error("HTTPServer: couldn't write content to socket.");
			}

			bzero(buffer, HTTPServer::BUFFER_SIZE);
			size = read(clientSocket, buffer, HTTPServer::BUFFER_SIZE - 1);
		}
	}
	catch (exception &e)
	{
		cerr << e.what() << endl;
	}

	try
	{

		shutdown(clientSocket, SHUT_RDWR);
		close(clientSocket);
	}
	catch (exception &e)
	{
		cerr << e.what() << endl;
	}
}
