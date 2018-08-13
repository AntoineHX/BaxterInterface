//! \example tutorial-detection-object-mbt2.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/vision/vpKeyPoint.h>

//Options
// #define IMATCHING
#define HYBRID_MATCHING
#define VAR_TRESH

#ifdef HYBRID_MATCHING
    enum State
    {
      WAITING_FOR_INITIALIZATION,
      TRACKING,
      LOST,
      UNCERTAIN //=Tracking OK / Matching FAIL
    };
#endif

#if (VISP_HAVE_OPENCV_VERSION >= 0x020400)
void learnCube(const vpImage<unsigned char> &I, vpMbGenericTracker &tracker, vpKeyPoint &keypoint_learning, int id)
{
  //! [Keypoints reference detection]
  std::vector<cv::KeyPoint> trainKeyPoints;
  double elapsedTime;
  keypoint_learning.detect(I, trainKeyPoints, elapsedTime);
  //! [Keypoints reference detection]

  //! [Keypoints selection on faces]
  std::vector<vpPolygon> polygons;
  std::vector<std::vector<vpPoint> > roisPt;
  std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = tracker.getPolygonFaces();
  polygons = pair.first;
  roisPt = pair.second;

  std::vector<cv::Point3f> points3f;
  vpHomogeneousMatrix cMo;
  tracker.getPose(cMo);
  vpCameraParameters cam;
  tracker.getCameraParameters(cam);
  vpKeyPoint::compute3DForPointsInPolygons(cMo, cam, trainKeyPoints, polygons, roisPt, points3f);
  //! [Keypoints selection on faces]

  //! [Keypoints build reference]
  keypoint_learning.buildReference(I, trainKeyPoints, points3f, true, id);
  //! [Keypoints build reference]

  //! [Display reference keypoints]
  for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
    vpDisplay::displayCross(I, (int)it->pt.y, (int)it->pt.x, 4, vpColor::red);
  }
  //! [Display reference keypoints]
}
#endif

#ifdef VAR_TRESH
//Compute cummulative variance of a list of vector. 
double varVector(std::list< vpArray2D<double> > vector_list)
{
  //Attention aux vecteurs vides
  double var[3], mean[3];

  //Compute means
  for (std::list< vpArray2D<double> >::iterator it=vector_list.begin(); it != vector_list.end(); ++it)
  {
    if((*it).size()!=3)
    {
      std::cout<<"WARNING varVector : invalid array size"<<std::endl;
      return -1;
    }
    else
    {
      for(unsigned int j=0; j< 3; j++)
      {
        mean[j]+=*(*it)[j];
      }
    }
  }
  for(unsigned int j=0; j<3; j++)
  {
    mean[j] /= vector_list.size();
  }

  //Compute variances
  for (std::list< vpArray2D<double> >::iterator it=vector_list.begin(); it != vector_list.end(); ++it)
  {
    for(unsigned int j=0; j< 3; j++)
    {
      var[j]+=(*(*it)[j]-mean[j])*(*(*it)[j]-mean[j]);
    }
  }
  for(unsigned int j=0; j<3; j++)
  {
    var[j] /= vector_list.size();
  }
  return var[0]+var[1]+var[2];
}
#endif

#ifdef HYBRID_MATCHING
float computeConfidenceLevel(std::list<State> states)
{
  float res = 0;

  for (std::list<State>::iterator it=states.begin(); it != states.end(); ++it)
  {
    switch(*it)
    {
      case TRACKING:
        res += 1;
        break;
      case UNCERTAIN:
        res += 0.5;
        break;
      case LOST:
        res -=1;
      case WAITING_FOR_INITIALIZATION:
        break;
    }
  }
  res = res/states.size();

  return res;
}
#endif

int main(int argc, char **argv)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020400)
  //! [MBT code]
  try {
    std::string videoname = "book.mpeg";

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--name")
        videoname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "\nUsage: " << argv[0] << " [--name <video name>] [--help]\n" << std::endl;
        return 0;
      }
    }
    std::string parentname = vpIoTools::getParent(videoname);
    std::string objectname = vpIoTools::getNameWE(videoname);

    if (!parentname.empty())
      objectname = parentname + "/" + objectname;

    std::cout << "Video name: " << videoname << std::endl;
    std::cout << "Tracker requested config files: " << objectname << ".[init,"
#ifdef VISP_HAVE_XML2
              << "xml,"
#endif
              << "cao or wrl]" << std::endl;
    std::cout << "Tracker optional config files: " << objectname << ".[ppm]" << std::endl;

    vpImage<unsigned char> I;
    vpHomogeneousMatrix cMo;
    vpCameraParameters cam;

    vpMbGenericTracker tracker; //Bug lors de la config de type KLT de tracker dans le constructeur.
    tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);

    bool usexml = false;
#ifdef VISP_HAVE_XML2
    if (vpIoTools::checkFilename(objectname + ".xml")) {
      tracker.loadConfigFile(objectname + ".xml");
      tracker.getCameraParameters(cam);
      usexml = true;
    }
#endif
    if (!usexml) {
      vpMe me;
      me.setMaskSize(5);
      me.setMaskNumber(180);
      me.setRange(7);
      me.setThreshold(5000);
      me.setMu1(0.5);
      me.setMu2(0.5);
      me.setSampleStep(4);
      me.setNbTotalSample(250);
      tracker.setMovingEdge(me);
      cam.initPersProjWithoutDistortion(547, 542, 339, 235);
      tracker.setCameraParameters(cam);
      tracker.setAngleAppear(vpMath::rad(89));
      tracker.setAngleDisappear(vpMath::rad(89));
      tracker.setNearClippingDistance(0.01);
      tracker.setFarClippingDistance(10.0);
      tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);

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

    tracker.setOgreVisibilityTest(false);
    if (vpIoTools::checkFilename(objectname + ".cao"))
      tracker.loadModel(objectname + ".cao");
    else if (vpIoTools::checkFilename(objectname + ".wrl"))
      tracker.loadModel(objectname + ".wrl");
    tracker.setDisplayFeatures(true);
    //! [MBT code]

    //! [Keypoint declaration]
    vpKeyPoint keypoint_learning("ORB", "ORB", "BruteForce-Hamming");
#if (VISP_HAVE_OPENCV_VERSION < 0x030000)
    keypoint_learning.setDetectorParameter("ORB", "nLevels", 1);
#else
    cv::Ptr<cv::ORB> orb_learning = keypoint_learning.getDetector("ORB").dynamicCast<cv::ORB>();
    if (orb_learning != NULL) {
      orb_learning->setNLevels(1);
    }
#endif
//! [Keypoint declaration]

#if defined(VISP_HAVE_X11)
    vpDisplayX display;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display;
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV display;
#else
    std::cout << "No image viewer is available..." << std::endl;
    return 0;
#endif


    videoname = "data_test5/%03d.jpg";
    /*
     * Start the part of the code dedicated to object learning from 3 images
     */
    unsigned int init_data_nb = 11;

    // vpVideoReader g;
    // g.setFileName("book_training/train_image_%03d.png");

    std::string trainData[] = {
      "book_training/train_image_000", 
      "book_training/train_image_001", 
      "book_training/train_image_002",
      "book_training/train_image_003",
      "book_training/train_image_004",
      "book_training/train_image_005",
      "book_training/train_image_006",
      "book_training/train_image_007",
      "book_training/train_image_008",
      "book_training/train_image_009",
      "book_training/train_image_010"};

    // std::string trainData[] = {
    //   "book2_training/train_image_000", 
    //   "book2_training/train_image_001",
    //   "book2_training/train_image_002",
    //   "book2_training/train_image_003"};

    // std::string trainData[] = {
      // "box_training/train_image_000", 
      // "box_training/train_image_001",
      // "box_training/train_image_002",
      // "box_training/train_image_003",
      // "box_training/train_image_004",
      // "box_training/train_image_005"};

    vpHomogeneousMatrix initPoseTab[init_data_nb];
    // std::cout<<"Load init cMo"<<std::endl;
    // vpArray2D<double>::loadYAML(cMoName[0],initPoseTab[0]);
    // initPoseTab[0].print();

    //Acquisition des cMo d'intialisation
    for (int i = 0; i < init_data_nb; i++)
    {
      // std::cout<<i<<std::endl;
      vpArray2D<double>::loadYAML(trainData[i]+".yml",initPoseTab[i]);
    }

    for (int i = 0; i < init_data_nb; i++) {
      //Acquisition des images d'initialisation
      vpImageIo::read(I, trainData[i]+".png");
      // if(i==0 || !g.end())
      //   g.acquire(I);
      // else
      //   std::cout<<"Missing training image !"<<std::endl;

      // vpArray2D<double>::loadYAML(cMoName[i],initPoseTab[i]);
      
      if (i == 0) {
        display.init(I, 10, 10);
      }
      std::stringstream title;
      title << "Learning cube on image: " << trainData[i];
      // title << "Learning cube on image: " << g.getFrameIndex();
      vpDisplay::setTitle(I, title.str().c_str());

      vpDisplay::display(I);

      //! [Set tracker pose]
      // tracker.setPose(I, initPoseTab[i]);
      tracker.initFromPose(I,initPoseTab[i]);
      //! [Set tracker pose]

      // std::cout<<"Refine"<<std::endl;
      //! [Refine pose]
      tracker.track(I);
      //! [Refine pose]

      // std::cout<<"Display"<<std::endl;
      //! [Display tracker pose]
      tracker.getPose(cMo);
      tracker.display(I, cMo, cam, vpColor::red);
      //! [Display tracker pose]

      // std::cout<<"Learn"<<std::endl;
      //! [Learn cube call]
      learnCube(I, tracker, keypoint_learning, i);
      //! [Learn cube call]

      vpDisplay::displayText(I, 10, 10, "Learning step: keypoints are detected on visible cube faces", vpColor::red);
      if (i < 2) {
        vpDisplay::displayText(I, 30, 10, "Click to continue the learning...", vpColor::red);
      } else {
        vpDisplay::displayText(I, 30, 10, "Click to continue with the detection...", vpColor::red);
      }

      vpDisplay::flush(I);
      // vpDisplay::getClick(I, true);
      std::cout<<trainData[i]<<" learned !"<<std::endl;
    }
    // g.close();

    //! [Save learning data]
    #ifdef IMATCHING
    keypoint_learning.saveLearningData("book_learning_data.bin", true, true); //+sauvegarde des images d'entrainement pour l'affichage
    #else
    keypoint_learning.saveLearningData("book_learning_data.bin", true, false);
    #endif
    //! [Save learning data]

    /*
     * Start the part of the code dedicated to detection and localization
     */
    //! [Init keypoint detection]
    vpKeyPoint keypoint_detection("ORB", "ORB", "BruteForce-Hamming");
#if (VISP_HAVE_OPENCV_VERSION < 0x030000)
    keypoint_detection.setDetectorParameter("ORB", "nLevels", 1);
#else
    cv::Ptr<cv::ORB> orb_detector = keypoint_detection.getDetector("ORB").dynamicCast<cv::ORB>();
    orb_detector = keypoint_detection.getDetector("ORB").dynamicCast<cv::ORB>();
    if (orb_detector != NULL) {
      orb_detector->setNLevels(1);
    }
#endif
    //! [Init keypoint detection]
    //! [Load teabox learning data]
    keypoint_detection.loadLearningData("book_learning_data.bin", true);
    //! [Load teabox learning data]

#ifdef IMATCHING
    //! [Create image matching]
    vpImage<unsigned char> IMatching;
    keypoint_detection.createImageMatching(I, IMatching);
    //! [Create image matching]
#endif

    // std::cout<<"Reading : "<<videoname<<std::endl;
    vpVideoReader g;
    g.setFileName(videoname);
    g.open(I);

#if defined VISP_HAVE_X11
    vpDisplayX display2;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display2;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display2;
#else
    vpDisplayOpenCV display2;
#endif

  #ifdef IMATCHING
    display2.init(IMatching, 50, 50, "Display matching between learned and current images");
  #endif
    vpDisplay::setTitle(I, "Cube detection and localization");

///// TEST ////
// if(rect_roi==NULL)
//   rect_roi = new vpRect();
// double time=0, nb=0;
    // display.init(I, 10, 10);

  #ifdef VAR_TRESH
    //Var Treshold
    std::list< vpArray2D<double> > translation_mem;
    std::list< vpArray2D<double> > rotation_mem;
    unsigned int var_mem_length = 15; // + = stabilité / - = rapidité
    double tvar_treshold = 0.3;
    double rvar_treshold = 0.5;
    double tvar, rvar;
    //Var Treshold
  #endif

  #ifdef HYBRID_MATCHING
    vpRect* rect_roi = new vpRect();
    State track_state = WAITING_FOR_INITIALIZATION;
    std::list<State> state_mem;
    unsigned int state_mem_length = 10;
  #endif


///////////////

    double error;
    bool click_done = false;

    while (!g.end()) {
      g.acquire(I);
      vpDisplay::display(I);

    #ifdef IMATCHING
      //! [Insert image matching]
      keypoint_detection.insertImageMatching(I, IMatching);
      //! [Insert image matching]

      vpDisplay::display(IMatching);
    #endif
      // vpDisplay::displayText(I, 10, 10, "Detection and localization in process...", vpColor::red);

      double elapsedTime;

      // ! [Matching and pose estimation]
      // if (keypoint_detection.matchPoint(I, cam, cMo, error, elapsedTime,NULL,*rect_roi)) {
      if (keypoint_detection.matchPoint(I, cam, cMo, error, elapsedTime)) {
        //! [Matching and pose estimation]
        // std::cout<<"Match !"<<std::endl;
        // std::cout<<"T :"<<std::endl<<cMo.getTranslationVector()<<std::endl<<"R :"<<std::endl<<cMo.getThetaUVector()<<std::endl;

      #ifdef VAR_TRESH
        //Var Treshold
        translation_mem.push_front(cMo.getTranslationVector());
        rotation_mem.push_front(cMo.getThetaUVector());
      
        if(translation_mem.size()==var_mem_length)
        {
          // tvar = varVector(translation_mem);
          // std::cout<<"Var trans : "<<tvar<<std::endl;
          translation_mem.pop_back();
        }
        if(rotation_mem.size()==var_mem_length)
        {
          // rvar=varVector(rotation_mem);
          // std::cout<<"Var rot : "<<rvar<<std::endl;
          rotation_mem.pop_back();
        }

        tvar = varVector(translation_mem);
        rvar=varVector(rotation_mem);
        if(tvar < tvar_treshold && rvar < rvar_treshold)
        {
        //Var Treshold
      #endif

      #ifdef HYBRID_MATCHING
        //Init ou Reinit du tracker après perte ou après incertitude
        if(track_state != TRACKING || (tvar<0.1 && rvar<0.1))
        {
          tracker.initFromPose(I,cMo);
          track_state = TRACKING;
        }
      #endif
        //! [Tracker set pose]
        // tracker.setPose(I, cMo);
        //! [Tracker set pose]

      /////TEST/////
      //   time+=elapsedTime; nb++;
      // std::vector<vpImagePoint> roi;
      // for(unsigned int i=0; i<tracker.getNbPolygon();i++)
      // {
      //     std::vector<vpImagePoint> temp = tracker.getPolygon(i)->getRoi(cam);
      //     roi.insert(roi.end(), temp.begin(), temp.end());
      // }
      // delete rect_roi;
      // rect_roi = new vpRect(roi);

        //Affichage de l'origine pointée par cMo
      // vpPoint origine;
      // origine.setWorldCoordinates(0,0,0) ;
      // // std::cout <<"Origne (o):"<< origine.oP << std::endl ;
      // origine.changeFrame(cMo) ;
      // // std::cout << "Origine (c) : "<< origine.cP << std::endl ;
      // origine.project(cMo);
      // // std::cout << "Origine (2D) : "<< origine.p << std::endl ;

      // vpImagePoint ImRes;
      // vpMeterPixelConversion::convertPoint(cam,origine.p[0],origine.p[1],ImRes);
      // // std::cout<<"Point : ";
      // // std::cout<<ImRes.get_u()<<" / "<<ImRes.get_v(); 
      // // std::cout<<std::endl;
      // vpDisplay::displayPoint(I,ImRes,vpColor::blue,10);
      /////////////

        //! [Display]
        // tracker.display(I, cMo, cam, vpColor::red, 2);
        vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 3);
#ifdef VAR_TRESH
        }
        else
        {
          std::cout<<"Object detection uncertain in frame : "<<g.getFrameIndex()<<std::endl;

        #ifdef HYBRID_MATCHING
          //Tentative d'améliorationde la détédction avec ROI
          if(track_state == TRACKING || track_state == UNCERTAIN) //Tracker initialisé
          {
            std::vector<vpImagePoint> roi;
            for(unsigned int i=0; i<tracker.getNbPolygon();i++)
            {
                std::vector<vpImagePoint> temp = tracker.getPolygon(i)->getRoi(cam);
                roi.insert(roi.end(), temp.begin(), temp.end());
            }
            delete rect_roi;
            rect_roi = new vpRect(roi);

            if (keypoint_detection.matchPoint(I, cam, cMo, error, elapsedTime,NULL,*rect_roi))
            {
              //Remplace previous result
              translation_mem.pop_front();
              rotation_mem.pop_front();
              translation_mem.push_front(cMo.getTranslationVector());
              rotation_mem.push_front(cMo.getThetaUVector());

              tvar = varVector(translation_mem);
              rvar=varVector(rotation_mem);
              if(tvar < tvar_treshold && rvar < rvar_treshold)
              {
                std::cout<<"Object detection improved : "<<g.getFrameIndex()<<std::endl;
                vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 3);
                track_state = TRACKING;
              }
              else 
              {
                track_state = UNCERTAIN;
              }
            }
            //BoF BOF
            // else if( track_state = WAITING_FOR_INITIALIZATION )
            // {
            //   track_state = UNCERTAIN;
            // }
          }
        #endif
        }
#endif

        //! [Display]

      // unsigned int nbMatch = keypoint_detection.matchPoint(I);
      // std::cout<<"Nb KeyPoint : "<<nbMatch<<std::endl;
      // if(nbMatch > 20)
      // {

      #ifdef IMATCHING
        keypoint_detection.displayMatching(I, IMatching);
      #endif

        //! [Get RANSAC inliers outliers]
        std::vector<vpImagePoint> ransacInliers = keypoint_detection.getRansacInliers();
        std::vector<vpImagePoint> ransacOutliers = keypoint_detection.getRansacOutliers();
        //! [Get RANSAC inliers outliers]

        //! [Display RANSAC inliers]
        for (std::vector<vpImagePoint>::const_iterator it = ransacInliers.begin(); it != ransacInliers.end(); ++it) {
          vpDisplay::displayCircle(I, *it, 4, vpColor::green);
        #ifdef IMATCHING
          vpImagePoint imPt(*it);
          imPt.set_u(imPt.get_u() + I.getWidth());
          imPt.set_v(imPt.get_v() + I.getHeight());
          vpDisplay::displayCircle(IMatching, imPt, 4, vpColor::green);
        #endif
        }
        //! [Display RANSAC inliers]

        //! [Display RANSAC outliers]
        for (std::vector<vpImagePoint>::const_iterator it = ransacOutliers.begin(); it != ransacOutliers.end(); ++it) {
          vpDisplay::displayCircle(I, *it, 4, vpColor::red);
        #ifdef IMATCHING
          vpImagePoint imPt(*it);
          imPt.set_u(imPt.get_u() + I.getWidth());
          imPt.set_v(imPt.get_v() + I.getHeight());
          vpDisplay::displayCircle(IMatching, imPt, 4, vpColor::red);
        #endif
        }
        //! [Display RANSAC outliers]

      #ifdef IMATCHING
        //! [Display image matching]
        keypoint_detection.displayMatching(I, IMatching);
        //! [Display image matching]


        //! [Display model image matching]
        // vpCameraParameters cam2;
        // cam2.initPersProjWithoutDistortion(cam.get_px(), cam.get_py(), cam.get_u0() + I.getWidth(),
        //                                    cam.get_v0() + I.getHeight());
        // tracker.setCameraParameters(cam2);
        // tracker.setPose(IMatching, cMo);
        // tracker.display(IMatching, cMo, cam2, vpColor::red, 2);
        // vpDisplay::displayFrame(IMatching, cMo, cam2, 0.05, vpColor::none, 3);
        //! [Display model image matching]
      #endif
      }
      else
      {
        std::cout<<"Object Lost in frame : "<<g.getFrameIndex()<<std::endl;
      #ifdef HYBRID_MATCHING
        track_state = UNCERTAIN;
      #endif
      }

    #ifdef HYBRID_MATCHING
      if(track_state == TRACKING || track_state == UNCERTAIN) //Tracker intialisé
      {
        try
        {
          tracker.track(I);
          // tracker.testTracking();
        }
        catch(...)
        {
          track_state = LOST;
          std::cout<<"Tracker : Object Lost in frame : "<<g.getFrameIndex()<<std::endl;
        }
      
        if(track_state !=LOST)
        { 
          tracker.getPose(cMo);
          tracker.display(I, cMo, cam, vpColor::blue, 4);
        }
      }
      state_mem.push_front(track_state);
      if(state_mem.size()==state_mem_length)
      {
        std::stringstream title2;
        title2 << "Confidence level :"<<computeConfidenceLevel(state_mem);
        // vpDisplay::displayText(I, 10, 10, "Confidence Level : "+std::to_string(computeConfidenceLevel(state_mem)), vpColor::red); //C++11
        vpDisplay::displayText(I, 10, 10, title2.str().c_str(), vpColor::red);
        // std::cout<<"Confidence level :"<<computeConfidenceLevel(state_mem)<<std::endl;
        state_mem.pop_back();
      }
    #endif

      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) {
        click_done = true;
        break;
      }
      // vpDisplay::getClick(I, true);

    #ifdef IMATCHING
      vpDisplay::displayText(IMatching, 30, 10, "A click to exit.", vpColor::red);
      vpDisplay::flush(IMatching);

      if (vpDisplay::getClick(IMatching, false)) {
        click_done = true;
        break;
      }
      // vpDisplay::getClick(IMatching, true);
    #endif
    }

    // std::cout<<"Moyenne temps execution : "<<time/nb<<std::endl;
    // delete rect_roi;

  #if defined IMATCHING
    if (!click_done)
      vpDisplay::getClick(IMatching);
  #endif

#ifdef VISP_HAVE_XML2
    vpXmlParser::cleanup();
#endif
#if defined(VISP_HAVE_COIN3D) && (COIN_MAJOR_VERSION == 3)
    SoDB::finish();
#endif
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV and rebuild ViSP to use this example." << std::endl;
#endif

  return 0;
}
