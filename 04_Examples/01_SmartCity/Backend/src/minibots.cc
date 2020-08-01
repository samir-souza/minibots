#include <cstdint>
#include <tracker.hpp>
#include <unistd.h>
#include <logger.hpp>
#include <config.hpp>

int main(int argc, char** argv){
    try {
        int fps = Config::get()->get<int>("fps");
        int width = Config::get()->get<int>("width");
        int height = Config::get()->get<int>("height");
        int camid = Config::get()->get<int>("camid");
        bool preview = Config::get()->get<bool>("preview");
        std::string usage = argv[0] + std::string(" [-f <int> -w <int> -h <int> -c <int> -p]");
        std::string serverAddress;
        bool ok = true;
        int opt;
        while (ok and (opt = getopt(argc, argv, ":cs:f:w:h:p")) != -1) {
            switch (opt) {
                case 'f': fps = atoi(optarg); break;
                case 'w': width = atoi(optarg); break;
                case 'h': height = atoi(optarg); break;
                case 'c': camid = atoi(optarg); break;
                case 'p': preview = true; break;
                case 's': serverAddress = optarg; break;
                default: std::cerr << usage << std::endl; ok = false;
            } // switch
        } // while
        if (ok) {
            std::cout << "MiniBots tracker v1.0. Launching..." << std::endl;
            Tracker tracker(width, height, camid, fps, preview, serverAddress);
            tracker.start();
		    std::cout << "Finished" << std::endl;
	    } // if
    } catch( const cv::Exception& ex ) {
        Logger::get()->log( Logger::ERROR, "minibots::main OpenCV exception - code[%d] error[%s] file[%s] func[%s] line[%d] msg[%s]", 
            ex.code, ex.err.c_str(), ex.file.c_str(), ex.func.c_str(), ex.line, ex.msg.c_str() );
    } catch( const std::runtime_error& ex ) {
        std::cerr << "Error! Check the log file" << std::endl;
        Logger::get()->log( Logger::ERROR, "minibots::main std::runtime_error - msg[%s]", ex.what() );
    } // catch
    return 0;
}
