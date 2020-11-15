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

/*
La marche à suivre :

installer l'app sur www.postman.com => ça facilite la génération du code qui permet de faire une requête
	créer un compte etc.

aller sur https://userportal.metsafecloud.com/login
	avec le token : 5lIxHQOL... XXXX

on va créer une requête qui permet de connaître les cellules orageuses
	
	sur la page metsafe, aller dans doc puis quickstart
	cliquer sur le bouton orange "Run in postman"
	
	ça devrait importer une "collection" dans postman

dans postman
	créer un environnement "MetSafe", puis ajouter la valeur du token

	dans la liste de collections, choisir core services > Web Feature Services > sample 1, le "get" en vert
		on laisse les paramètres par défaut, mais en gros faut donner une bounding box

	appuyer sur send pour voir à quoi ressemble le résultat: du texte json (je n'ai pas trouvé comment lui demander du xml)

	à droite y'a un lien 'code' en orange, cliquer dessus
	choisir 'C - libcurl'. C'est ce code qu'on va prendre, puisque djnn-cpp utilise déjà libcurl
*/

_native_code_
%{
#include "exec_env/global_mutex.h"

#include <nlohmann/json.hpp>

namespace curl { // fix 'Rectangle' clash name for windowss
#include <curl/curl.h>
#include <curl/easy.h>
}

size_t
mycurl_write_callback(char *ptr, size_t size, size_t nmemb, void *userdata)
{
    //std::cerr << "curl received " << size * nmemb << " bytes." << std::endl;
    //std::cerr << size << " " << nmemb << " " << ptr << std::endl;
    std::string * content = reinterpret_cast<std::string*>(userdata);
    content->append (ptr, size*nmemb);
    return size * nmemb;
}


void
cpp_action (Process* c)
{
    std::string content;

    using namespace curl;
    // code pasted from postman
    CURL *curl;
    CURLcode res;
    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "GET");
        std::string url = "https://apiv3.metsafecloud.com/core/wfs?service=WFS&token=";
        url += getenv ("METSAFE_TOKEN");
        url += "&version=2.0.0&request=GetFeature&typeName=metgate-replay:lightningcells_meteorage&outputFormat=json&count=50&bbox=41.57,-5.27,51.67,9.66&srsName=EPSG:4326&sortBy=startValidity+D";
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(curl, CURLOPT_DEFAULT_PROTOCOL, "https");
        struct curl_slist *headers = NULL;
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        // addition to postman code
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, mycurl_write_callback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &content);

        // blocking call
        res = curl_easy_perform(curl);
    }
    curl_easy_cleanup(curl);
    if (content.empty()) return;

    //std::cerr << content.size() << std::endl;
    //std::cerr << content << std::endl;
    vector<vector<vector<float>>> polys;
    using json = nlohmann::json;
    //try {
        auto j = json::parse (content);
        for (auto feature: j["features"]) {
            auto g = feature["geometry"];
            if(g["type"]=="MultiPolygon") {
                auto cs = g["coordinates"];
                //std::cerr << cs << std::endl;
                auto c = cs.get<vector<vector<vector<vector<float>>>>>();
                polys.push_back (c[0][0]);
            }
        }
    //} catch (nlohmann::detail::parse_error&e) {
    //}
    

    // first create all djnn objects before updating the tree
    // to minimize the time spent with exclusive access

    // inappropriate for loading of large set of data:
    // should be incremental, with SAX json parser TODO
    // cf Darius way of doing that?

    //Process * tmp = new Process (nullptr, ""); // TODO when Process::move_children_from (Process*) is available
    Process * tmp = nullptr;
    std::vector<Process*> processes; // use a vector until Process::move_children_from (Process*) is available

    int numpoly=0;
    for (auto &poly: polys) {
        auto * djnnpoly = new Polyline (tmp, std::to_string(numpoly++));
        int i=0;
        for (auto& coord: poly) {
            //std::cerr << coord[0] << " " << coord[1] << " " ;
            new PolyPoint (djnnpoly, std::to_string(i++), coord[0], coord[1]);
        }
        processes.push_back(djnnpoly);
        //std::cerr << std::endl;
    }

    // second update the djnn tree

    get_exclusive_access(DBG_GET);

    // To get the source that triggered the native action:
    //Process *source = c->get_activation_source ();
    // To get the user_data
    Process *data = (Process*) get_native_user_data (c);

    Process *meteo = data->find_child ("meteo");
    //std::cerr << meteo << std::endl;
    // meteo->move_children_from (tmp); // TODO when Process::move_children_from (Process*) is available
    
    numpoly = 0;
    for (auto p: processes) {
        meteo->add_child(p, std::to_string(numpoly++));
    }

    release_exclusive_access(DBG_REL);
    
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

    Translation _(0,-2500)
    Scaling _(70,70, 0,0)

    NoFill _
    OutlineColor _(#FF00FF)

    Component meteo {
    }

    // Bind a C++ native action
	NativeAsyncAction cpp_na (cpp_action, root, 1)

    // since we apply set_value to t in the native,
    // we specify a causality relation between the native and the text it uses
    cpp_na ~> t

    //Rectangle rr (10,10,10,10)
    //rr.press -> (root) { dump root.meteo }

    btn.click -> cpp_na
    btn.click -> {"STARTED" =: t.text}
}

