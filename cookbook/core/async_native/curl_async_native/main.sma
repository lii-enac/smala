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

import gui.widgets.Button
import gui.interactors.SimpleDrag

_native_code_
%{
#include "exec_env/global_mutex.h"

namespace curl { // fix 'Rectangle' clash name for windowss
#include <curl/curl.h>
#include <curl/easy.h>
}

size_t
mycurl_write_callback(char *ptr, size_t size, size_t nmemb, void *userdata)
{
    //std::cerr << "curl received " << size * nmemb << " bytes." << std::endl;
    //std::cerr << ptr << std::endl;
    std::string * content = reinterpret_cast<std::string*>(userdata);
    *content += ptr;
    return size * nmemb;
}

void
cpp_action (Process* c)
{
    using namespace curl;
    CURL *curl = curl_easy_init ();
    if (!curl) {
        std::cerr << "error setting curl" << std::endl;
        return;
    }

    std::string uri = "https://www.lemonde.fr/rss/une.xml";
    //std::string uri = "https://djnn.net/wp-content/uploads/2016/07/EICS_demo_2014.mp4";

    curl_easy_setopt(curl, CURLOPT_URL, uri.c_str ());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, mycurl_write_callback);
    //curl_easy_setopt(curl, CURLOPT_VERBOSE, 1);
    std::string content;
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &content);

    std::cerr << "curl getting " << uri << std::endl;

    // blocking call, only once here since the URI is finished (and there is no curl 'mainloop')
    auto res = curl_easy_perform (curl);
    if (res != CURLE_OK) {
        std::cerr << "error performing curl" << std::endl;
        return;
    }

    curl_easy_cleanup (curl);

    //std::cerr << content << std::endl;
    auto beg = content.find("<item>");
    beg = content.find("<title>", beg);
    auto end = content.find("</title>", beg+7);
    string news = content.substr(beg+7,end-(beg+7));
    std::cerr << news << std::endl;
    
    
    // update djnn tree

    get_exclusive_access(DBG_GET);
    
    // To get the source that triggered the native action:
	//Process *source = c->get_activation_source ();
	
	// To get the user_data
 	Process *data = (Process*) get_native_user_data (c);
    
 	Process *t = data->find_child ("t");
    ((Text*) t)->text()->set_value (news, 1) ;

    release_exclusive_access(DBG_REL);

    // exiting, _end will be activated and graph will be executed
}

%}

_main_
Component root {

	Frame f ("f", 0, 0, 500, 600)
    Exit ex (0, 1)
    f.close -> ex
    FillColor fcc (#000000)
    Text explanation1 (10, 20, "Click the button to launch the async action")
    Text explanation2 (10, 40, "then, drag the rectangle to check that the application is not freezed")
    Text explanation3 (10, 60, "When the action is terminated, the title of the online ressource should appear")
    Text t (10, 120, "  ")
    Text t2 (10, 140, "")
    Button btn (f, "launch", 50, 150)
    FillColor fc (#FF00FF)
    Rectangle r (200, 200, 100, 100, 0, 0)
    Ref toDrag (r)
    SimpleDrag _ (toDrag, f)

    // Bind a C++ native action
	NativeAsyncAction cpp_na (cpp_action, root, 1)

    // since we apply set_value to t in the native,
    // we specify a causality relation between the native and the text it uses
    cpp_na ~> t

    btn.click -> cpp_na
    btn.click -> {"STARTED" =: t.text}
}

