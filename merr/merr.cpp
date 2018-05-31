#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

using namespace std;

enum class error {
  source=1,
  dest=2,
  tempfile=3,
  malformed=4,
  noerror=5
};

void
print_usage(const string& progname)
{
  cout << progname << ": " << endl;
}

// https://stackoverflow.com/a/478960
std::string
exec(const string& cmd) {
  std::array<char, 128> buffer;
  std::string result;
  std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe) throw std::runtime_error("popen() failed!");
  while (!feof(pipe.get())) {
    if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
      result += buffer.data();
  }
  return result;
}

int
main(int argc, const char* argv[])
{
  string progname = argv[0];
  string source, dest, compiler;
  for (int i=0; i<argc; ++i) {
    if(string(argv[i])=="-o") {
      ++i;
      dest = argv[i];
    } else if(string(argv[i])=="-c") {
      ++i;
      compiler = argv[i];
    } else {
      source = argv[i];
    }
  }
  //cout << source << " " << dest << " " << compiler << endl;

  ifstream ifs(source);
  if(!ifs.good()) {
    cerr << progname << ": could not open source file " << source << endl;
    return (int) error::source;
  }
  string content(istreambuf_iterator<char>(ifs), {});
  //cout << content << endl;

  ofstream ofs(dest);
  if(!ifs.good()) {
    cerr << progname << ": could not open dest file " << dest << endl;
    return (int) error::dest;
  }

  bool malformed=false;
  bool noerror=false;
  int line=0;
  string code;

  for (size_t i=0; i<content.size();) {
    // parse next example and message
    size_t beg_code = i;
    size_t end_code = content.find(":::", i);
    if(end_code==string::npos) { malformed=true; break; }
    size_t beg_msg = end_code + 3;
    size_t end_msg = content.find("\n", beg_msg);
    if(end_msg==string::npos) {
      end_msg = content.size();
      i = end_msg;
    }
    else {
      i = end_msg+1; // after \n
    }
    code = content.substr(beg_code, end_code-beg_code); //cout << "--" << code << "--" << endl;
    string msg = content.substr(beg_msg, end_msg-beg_msg); //cout << "**" << msg << "**" << endl;

    // create temporary source file containing example code
    // https://stackoverflow.com/a/1022545
    char filename[] = "/tmp/smala_merrXXXX.sma"; // template for our file.        
    int fd = mkstemp(filename);    // Creates and opens a new temp file r/w. Xs are replaced with a unique number.
    if (fd == -1) {
      cerr << progname << ": could not create temporary file " << filename << endl;
      return (int) error::tempfile;        // Check we managed to open the file.
    }

    write(fd, code.c_str(), code.size());
    close(fd);

    // compile example code
    string cmd = compiler + " " + string(filename) + " 2>&1"; //cout << cmd << endl;
    string result = exec(cmd); //cout  << result << endl;
    
    unlink(filename); // get rid of temporary file

    // parse results
    size_t op = result.find("(");
    if(op==string::npos) { noerror=true; break; }
    size_t cp = result.find(")");
    string line_state = result.substr(op+1, cp-op-2); //cout << line_state << endl;
    int line, state;
    char comma;
    istringstream(line_state) >> line >> comma >> state; //cout << line << " " << state << endl;

    // update destination errors.h file 
    ofs << "ERR(" << line << ", " << state << ", \"" << msg << "\")" << endl;
  }

  if(malformed) {
    cerr << progname << ": malformed error file " << source << endl;
    return (int) error::malformed;
  }
  if(noerror) {
    cerr << progname << ": no error in example " << code << endl;
    return (int) error::noerror;
  }
  return 0;
}
