/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2018)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Stephane Conversy <stephane.conversy@enac.fr>
 *
 */

use core
use base
use display
use gui

_native_code_
%{
#include "exec_env/global_mutex.h"

#include <curl/curl.h>
#include <curl/easy.h>

size_t
mycurl_write_callback(char *ptr, size_t size, size_t nmemb, void *userdata)
{
    std::cerr << "curl received " << size * nmemb << " bytes." << std::endl;
    std::cerr << ptr << std::endl;
    return size * nmemb;
}

void
cpp_action (Process* c)
{
	// To get the source that triggered the native action:
	//Process *source = c->get_activation_source ();
	
	// To get the user_data:
 	//Process *data = (Process*) get_native_user_data (c);
 	//Process *fc = data->find_child ("fc");
    //((IntProperty*) fc->find_child ("value"))->set_value (0xFF0000, 1)  ;

    CURL *curl = curl_easy_init ();
    if (!curl) {
        std::cerr << "error setting curl" << std::endl;
        return;
    }

    //std::string uri = "http://smala.io";
    std::string uri = "https://www.lemonde.fr";
    //std::string uri = "https://djnn.net/wp-content/uploads/2016/07/EICS_demo_2014.mp4";

    curl_easy_setopt(curl, CURLOPT_URL, uri.c_str ());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, mycurl_write_callback);
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 1);
    //curl_easy_setopt(curl, CURLOPT_WRITEDATA, &d);

    std::cerr << "curl getting " << uri << std::endl;
    auto res = curl_easy_perform (curl);
    if (res != CURLE_OK) {
        std::cerr << "error performing curl" << std::endl;
        return;
    }

    curl_easy_cleanup (curl);

    // to print the component tree:
 	//fc->dump(0);

    get_exclusive_access(DBG_GET);
    // TODO: update djnn tree
    release_exclusive_access(DBG_REL);
}

%}

_main_
Component root {

	Frame f ("f", 0, 0, 500, 600)
    Exit ex (0, 1)
    f.close -> ex

    // Bind a C++ native action
	NativeAction cpp_na (cpp_action, root, 1)

    FillColor fc(255,0,0)
    Rectangle r(0,0,50,50)

	r.press -> cpp_na
}

