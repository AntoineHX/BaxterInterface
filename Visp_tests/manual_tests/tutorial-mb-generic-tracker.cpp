//! \example tutorial-mb-generic-tracker.cpp
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
//! [Include]
#include <visp3/mbt/vpMbGenericTracker.h>
//! [Include]
#include <visp3/io/vpVideoReader.h>

int main(int argc, char **argv)
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)

  try {
    std::string opt_videoname = "teabox.mpg";
    std::string opt_modelname = "teabox";
    int opt_tracker = 1;

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--video")
        opt_videoname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--model")
        opt_modelname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--tracker")
        opt_tracker = atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "\nUsage: " << argv[0]
                  << " [--video <video name>] [--model <model name>] "
                     "[--tracker <0=egde|1=keypoint|2=hybrid>] [--help]\n"
                  << std::endl;
        return 0;
      }
    }
    std::string parentname = vpIoTools::getParent(opt_modelname);
    std::string objectname = vpIoTools::getNameWE(opt_modelname);

    if (!parentname.empty())
      objectname = parentname + "/" + objectname;

    std::cout << "Video name: " << opt_videoname << std::endl;
    std::cout << "Tracker requested config files: " << objectname << ".[init, cao]" << std::endl;
    std::cout << "Tracker optional config files: " << objectname << ".[ppm]" << std::endl;

    //! [Image]
    vpImage<unsigned char> I;
    vpCameraParameters cam;
    //! [Image]
    //! [cMo]
    vpHomogeneousMatrix cMo;
    //! [cMo]

    vpVideoReader g;
    g.setFileName(opt_videoname);
    g.open(I);

    vpDisplay *display = NULL;
#if defined(VISP_HAVE_X11)
    display = new vpDisplayX;
#elif defined(VISP_HAVE_GDI)
    display = new vpDisplayGDI;
#else
    display = new vpDisplayOpenCV;
#endif
    display->init(I, 100, 100, "Model-based tracker");

    //! [Constructor]
    vpMbGenericTracker tracker;
    if (opt_tracker == 0)
      tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER);
#ifdef VISP_HAVE_MODULE_KLT
    else if (opt_tracker == 1)
      tracker.setTrackerType(vpMbGenericTracker::KLT_TRACKER);
    else
      tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
#else
    else {
      std::cout << "klt and hybrid model-based tracker are not available since visp_klt module is not available. "
                   "In CMakeGUI turn visp_klt module ON, configure and build ViSP again."
                << std::endl;
      return 0;
    }
#endif
    //! [Constructor]

    //! [Set parameters]
    if (opt_tracker == 0 || opt_tracker == 2) {
      vpMe me;
      me.setMaskSize(5);
      me.setMaskNumber(180);
      me.setRange(8);
      me.setThreshold(10000);
      me.setMu1(0.5);
      me.setMu2(0.5);
      me.setSampleStep(4);
      tracker.setMovingEdge(me);
    }

#ifdef VISP_HAVE_MODULE_KLT
    if (opt_tracker == 1 || opt_tracker == 2) {
      vpKltOpencv klt_settings;
      klt_settings.setMaxFeatures(300);
      klt_settings.setWindowSize(5);
      klt_settings.setQuality(0.015);
      klt_settings.setMinDistance(8);
      klt_settings.setHarrisFreeParameter(0.01);
      klt_settings.setBlockSize(3);
      klt_settings.setPyramidLevels(3);
      tracker.setKltOpencv(klt_settings);
      tracker.setKltMaskBorder(5);
    }
#endif

    //! [Set camera parameters]
    cam.initPersProjWithoutDistortion(839, 839, 325, 243);
    //! [Set camera parameters]
    tracker.setCameraParameters(cam);
    //! [Set parameters]

    //! [Load cao]
    tracker.loadModel(objectname + ".cao");
    //! [Load cao]
    //! [Set display]
    tracker.setDisplayFeatures(true);
    //! [Set display]
    //! [Init]
    tracker.initClick(I, objectname + ".init", true);
    //! [Init]

//////TEST//////
vpImage<unsigned char> ImRoi;
vpDisplay* display2 = new vpDisplayX;
// display2->init(ImRoi);
////////////////


    while (!g.end()) {
      g.acquire(I);
      vpDisplay::display(I);
      //! [Track]
      tracker.track(I);
      //! [Track]
      //! [Get pose]
      tracker.getPose(cMo);
      //! [Get pose]
      // std::cout<<"cMo : "; cMo.print(); std::cout<<std::endl;

      ////TEST////
      // vpPoint origine;
      // origine.setWorldCoordinates(0,0,0) ;
      // std::cout <<"Origne (o):"<< origine.oP << std::endl ;
      // origine.changeFrame(cMo) ;
      // std::cout << "Origine (c) : "<< origine.cP << std::endl ;
      // origine.project(cMo);
      // std::cout << "Origine (2D) : "<< origine.p << std::endl ;
      //vpPoint res = cMo * origine;
      // vpTranslationVector res = cMo.inverse()*torigine;
      // std::cout<<"Point : ";
      // res.print();
      //std::cout<<res.get_x()<<" / "<<res.get_y(); 
      // std::cout<<res[0]<<" / "<<res[1]<<" / "<<res[2];
      // std::cout<<std::endl;
      // std::cout<<"Polygon Nb : "<< tracker.getNbPolygon()<<std::endl;
      // for(unsigned int i=0; i<tracker.getNbPolygon();i++)
      // {
      //   tracker.getPolygon(i).getRoi(cam,cMo);
      // }


      // vpImagePoint ImRes;
      std::vector<vpImagePoint> roi;
      // roi = tracker.getPolygon(0)->getRoi(cam,cMo);
      // tracker.getPolygon(0)->computePolygonClipped();
      // tracker.getPolygon(0)->getRoiClipped(cam,roi);
      for(unsigned int i=0; i<tracker.getNbPolygon();i++)
      {
        // if(tracker.getPolygon(i)->isVisible())
        // { 
          std::vector<vpImagePoint> temp = tracker.getPolygon(i)->getRoi(cam);
          roi.insert(roi.end(), temp.begin(), temp.end());
        // }
      }
      // vpMeterPixelConversion::convertPoint(cam,origine.p[0],origine.p[1],ImRes);
      // std::cout<<"Point : ";
      // std::cout<<ImRes.get_u()<<" / "<<ImRes.get_v(); 
      // std::cout<<std::endl;
      // vpDisplay::displayPoint(I,ImRes,vpColor::green,10);

      //Création de l'imageRoi
      vpRect rect_roi(roi);

      // vpPolygon3D::roiInsideImage(ImRoi,roi);
      // std::cout<<"crop"<<std::endl;
      vpDisplay::close(ImRoi);
      vpImageTools::crop(I, rect_roi, ImRoi);
      if(!display2->isInitialised ())
      {
        display2->init(ImRoi,-1,-1,"ROI");
      }
      // ImRoi.resize(I.getHeight(),I.getWidth(),0);
      vpDisplay::displayRectangle(I,rect_roi, vpColor::green);

      //Affichage
      for(unsigned int i=0; i<roi.size();i++)
      {
        vpDisplay::displayPoint(I,roi[i],vpColor::green,10);
      }

       // vpDisplay::displayFrame(ImRoi, cMo, cam, 0.025, vpColor::none, 3);
      vpDisplay::display(ImRoi);
      vpDisplay::flush(ImRoi);
      ////////////

      //! [Display]
      tracker.getCameraParameters(cam);
      tracker.display(I, cMo, cam, vpColor::red, 2, true);
      //! [Display]
      vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
      vpDisplay::displayText(I, 10, 10, "A click to exit...", vpColor::red);

      vpDisplay::flush(I);

      if (vpDisplay::getClick(I, false))
        break;
    }
    vpDisplay::getClick(I);

    // vpImageIo::write(ImRoi, "ImRoi.png");

    //! [Cleanup]
    delete display;
    //! [Cleanup]
  } catch (const vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV and rebuild ViSP to use this example." << std::endl;
#endif
}
