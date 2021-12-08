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

#include <sstream>
#include <fstream>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace curl { // fix 'Rectangle' clash name for windowss
    #include <curl/curl.h>
    #include <curl/easy.h>
}

#include "JSONSaxParser.h"

size_t
mycurl_write_callback(char *ptr, size_t size, size_t nmemb, void *userdata)
{
    assert(ptr);
    //std::cerr << "curl received " << size * nmemb << " bytes." << std::endl;
    //std::cerr << size << " " << nmemb << " " << ptr << std::endl;
    std::string * content = reinterpret_cast<std::string*>(userdata);
    content->append (ptr, size*nmemb);
    return size * nmemb;
}

void
cpp_action (Process* c)
{
    const char* token = getenv ("METSAFE_TOKEN");
    std::string content;

    if (!token) {
        std::cerr << "no METSAFE_TOKEN env variable, please provide it to download remote data" << std::endl;
        std::cerr << "resorting to local file metsafe_result.json" << std::endl;
        std::ifstream ifs("metsafe_result.json");
        std::stringstream buffer;
        buffer << ifs.rdbuf();
        content = buffer.str();
    } else {
        using namespace curl;
        // code pasted from postman
        CURL *curl;
        CURLcode res;
        curl = curl_easy_init();
        if(curl) {
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "GET");
            std::string url = "https://apiv3.metsafecloud.com/core/wfs?service=WFS&token=";
            url += token;
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
    }

    //std::cerr << content.size() << std::endl;
    //std::cout << content << std::endl;

#if 1
    // smala/djnn-based json parsing

    get_exclusive_access(DBG_GET);
    // To get the source that triggered the native action:
    //Process *source = c->get_activation_source ();
    // To get the user_data
    auto * root = reinterpret_cast<Process*>(get_native_user_data (c));
    auto * parser = dynamic_cast<JSONSaxParser*>(root->find_child ("parser"));
    release_exclusive_access(DBG_REL);

    assert(parser);
    parser->parse(content);
    
#else
    // c++-based json parsing

    // fist find out relevant information in json content
    // inappropriate for loading of large set of data:
    // should be incremental

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
    
    // second create all djnn objects before updating the tree
    // to minimize the time spent with exclusive access

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


    // third update the djnn tree

    get_exclusive_access(DBG_GET);

    // To get the source that triggered the native action:
    // Process *source = c->get_activation_source ();
    // To get the user_data
    Process *data = reinterpret_cast<Process*>(get_native_user_data (c));
    Process * meteo = data->find_child ("meteo");
    assert(meteo);
    //std::cerr << meteo << std::endl;
    // meteo->move_children_from (tmp); // TODO when Process::move_children_from (Process*) is available
    
    numpoly = 0;
    for (auto p: processes) {
        meteo->add_child(p, std::to_string(numpoly++));
    }
    GRAPH_EXEC;

    release_exclusive_access(DBG_REL);
#endif
    
}


%}

_main_
Component root {
	Frame f ("f", 0, 0, 1100, 600)
    Exit ex (0, 1)
    f.close -> ex
    FillColor fcc (#000000)
    Text explanation1 (10, 20, "Click the button to launch the async action")
    Text explanation2 (10, 40, "then, drag the rectangle to check that the application is not freezed")
    Text explanation3 (10, 60, "When the action is terminated, the title of the online ressource should appear")
    Text t (10, 120, "  ")
    Text t2 (10, 140, "")
    Button btn (f, "launch", 50, 150)

    Rectangle r (200, 200, 100, 100, 0, 0)
    Ref toDrag (r)
    SimpleDrag _ (toDrag, f)

    // create polys in an invisible layer, to avoid displaying their construction
    Switch deactivated (init) {
        Component _
        Component tmp_layer
    }


    // reading polygons...

    TextPrinter tp

    // JSONSaxParser parser("array/array/number") // should be this but our implementation of xpath is not powerful enough
    JSONSaxParser parser("geometry/coordinates/array/array/array/array/number")

    //json_poly aka parser.geometry
    json_poly aka parser.geometry.coordinates.array.array.array
    json_point aka parser.geometry.coordinates.array.array.array.array
    json_coord aka parser.geometry.coordinates.array.array.array.array.number

    Ref poly (null)
    Ref point (null)
    DerefDouble xr (point, "x", DJNN_GET_ON_CHANGE)
    DerefDouble yr (point, "y", DJNN_GET_ON_CHANGE)

    json_poly -> (root) {
        //root.tp.input = "create poly"
        addChildrenTo root.deactivated.tmp_layer {
            Polyline poly
        }
    }

        json_point -> (root) {
            //root.tp.input = "add point"
            addChildrenTo root.deactivated.tmp_layer.poly.points {
                PolyPoint point (0,0)
                root.point = &point
            }
        }

            // we receive arrays of unspecified coordinates [coord1,coord2]
            // the following switches back and forth between assigning a received value into x or y coordinate
            Spike xy_switch_read

            json_coord -> xy_switch_read
            
            xys aka xy_switch_read
            FSM xy {
                State a
                State x {
                    //"set x" =: tp.input
                    json_coord.value =: xr.value
                }
                State y {
                    //"set y" =: tp.input
                    json_coord.value =: yr.value
                }
                a->x(xys)
                x->y(xys)
                y->x(xys)
            }


    json_poly !-> (root) {
        //root.tp.input = "move poly to a visible layer"
        addChildrenTo root.meteo {
            poly << root.deactivated.tmp_layer.poly
        }
    }
    
    Translation _(-300,-5800)
    Scaling _(140,140, 0,0)

    //OutlineWidth _(0) // 0 == do not apply scale to OutlineWidth in Qt
    OutlineWidth _(1/140) // should work on other renderer since 1/140 is 0!

    NoFill _
    OutlineColor _(#FF00FF)

    Component meteo // visible layer to be populated with polygons

    // Bind to native action to get the data and parse them
	NativeAsyncAction cpp_na (cpp_action, root, 1)
    btn.click -> cpp_na
    btn.click -> {"STARTED" =: t.text}

    // since we apply set_value to t in the native,
    // we specify a causality relation between the native and the text it uses
    cpp_na ~> t
    // idem for json
    cpp_na ~> json_poly // FIXME there should be a multi destination for ~>
    cpp_na ~> json_point
    cpp_na ~> json_coord

    //cpp_na.end -> (root) {
    //    dump root.meteo
    //    dump root.tp
    //}
}














/*
json sax state-machine could be
features[object]/geometry[object]/MultiPolygon[object]/coordinates[array];

FSM json_sax {
    FSM features {
        FSM geometry {
            FSM MultiPolygon {
                coordinates {

                }
            }
        }
    }
}

List {
    Component features {
        Component geometry {
            Component MultiPolygon {
                Component coordinates {
                    List _ {
                        List _ {
                            List pair {
                                float f
}   }   }   }   }   }   }   }

Ref refp

XPath xp("/features/geometry/MultiPolygon/coordinates")

xp.MultiPolygon -> {
    Polygon p ()
    p =: refp
}

features.geometry.MultiPolygon -> (root) {
    Polygon p ()
    p =: refp
}

features.geometry.MultiPolygon.coordinates._._.pair -> (root) {
    PolyPoint p($refp, [0],[1])
    features.geometry.MultiPolygon.coordinates._._._.f 
}


FSM json_sax {
    State features
    State geometry
    State MultiPolygon
    State coordinates 
    features -> geometry (sax.features.start)
    geometry -> features (sax.features.end)
}


*/



