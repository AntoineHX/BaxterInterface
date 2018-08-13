//! \example tutorial-detection-object-mbt2.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/vision/vpKeyPoint.h>

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

int main(int argc, char **argv)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020400)
  //! [MBT code]
  try {
    std::string videoname = "book.mpeg";
    std::string videoname2 = "book2.mpeg";

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

    std::string parentname2 = vpIoTools::getParent(videoname2);
    std::string objectname2 = vpIoTools::getNameWE(videoname2);

    if (!parentname.empty())
      objectname = parentname + "/" + objectname;

    if (!parentname2.empty())
      objectname2 = parentname2 + "/" + objectname2;

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

    vpImage<unsigned char> I2;
    vpHomogeneousMatrix cMo2;

    vpMbGenericTracker tracker(vpMbGenericTracker::EDGE_TRACKER);
    vpMbGenericTracker tracker2(vpMbGenericTracker::EDGE_TRACKER);

    bool usexml = false;
#ifdef VISP_HAVE_XML2
    if (vpIoTools::checkFilename(objectname + ".xml")) {
      tracker.loadConfigFile(objectname + ".xml");
      tracker2.loadConfigFile(objectname + ".xml");
      tracker.getCameraParameters(cam);
      usexml = true;
    }
#endif

    tracker.setOgreVisibilityTest(false);
    tracker2.setOgreVisibilityTest(false);

    if (vpIoTools::checkFilename(objectname + ".cao"))
      tracker.loadModel(objectname + ".cao");
    else if (vpIoTools::checkFilename(objectname + ".wrl"))
      tracker.loadModel(objectname + ".wrl");
    tracker.setDisplayFeatures(true);

    if (vpIoTools::checkFilename(objectname2 + ".cao"))
      tracker2.loadModel(objectname2 + ".cao");
    tracker2.setDisplayFeatures(true);
    //! [MBT code]

    //! [Keypoint declaration]
    vpKeyPoint keypoint_learning("ORB", "ORB", "BruteForce-Hamming");
    keypoint_learning.setDetectorParameter("ORB", "nLevels", 1);

    vpKeyPoint keypoint_learning2("ORB", "ORB", "BruteForce-Hamming");
    keypoint_learning2.setDetectorParameter("ORB", "nLevels", 1);

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

    /*
     * Start the part of the code dedicated to object learning from 3 images
     */
    //Acquisition des images d'initialisation
    std::string imageName[] = {
      "book_training/train_image_000.png", 
      "book_training/train_image_001.png",
      "book_training/train_image_002.png",
      "book_training/train_image_003.png",
      "book_training/train_image_004.png",
      "book_training/train_image_005.png",
      "book_training/train_image_007.png",
      "book_training/train_image_008.png"};
    // vpVideoReader g;
    // g.setFileName("book_training/train_image_%03d.png");
    std::string imageName2[] = {
      "book2_training/train_image_000.png", 
      "book2_training/train_image_001.png",
      "book2_training/train_image_002.png",
      "book2_training/train_image_003.png"};

    //Acquisition des cMo d'intialisation
    std::string cMoName[] = {
      "book_training/train_image_000.yml", 
      "book_training/train_image_001.yml", 
      "book_training/train_image_002.yml",
      "book_training/train_image_003.yml",
      "book_training/train_image_004.yml",
      "book_training/train_image_005.yml",
      "book_training/train_image_007.yml",
      "book_training/train_image_008.yml"};

      std::string cMoName2[] = {
      "book2_training/train_image_000.yml", 
      "book2_training/train_image_001.yml", 
      "book2_training/train_image_002.yml",
      "book2_training/train_image_003.yml"};

    vpHomogeneousMatrix initPoseTab[8];
    vpHomogeneousMatrix initPoseTab2[4];
    // std::cout<<"Load init cMo"<<std::endl;
    // vpArray2D<double>::loadYAML(cMoName[0],initPoseTab[0]);
    // initPoseTab[0].print();

    // std::fstream file;
    // file.open(cMoName2[1].c_str(), std::fstream::in);
 
    //  if (!file) {
    //   file.close();
    //   return false;
    //  }
    for (int i = 0; i < 8; i++)
    {
      vpArray2D<double>::loadYAML(cMoName[i],initPoseTab[i]);
      std::cout<<cMoName[i]<<" loaded !"<<std::endl;
    }

    for (int i = 0; i < 4; i++)
    {
      vpMatrix::loadMatrixYAML(cMoName2[i],initPoseTab2[i]);
      std::cout<<cMoName2[i]<<" loaded !"<<std::endl;
    }

    //Tracker 1
    for (int i = 0; i < 8; i++) {
      vpImageIo::read(I, imageName[i]);
      // if(i==0 || !g.end())
      //   g.acquire(I);
      // else
      //   std::cout<<"Missing training image !"<<std::endl;

      // vpArray2D<double>::loadYAML(cMoName[i],initPoseTab[i]);
      
      if (i == 0) {
        display.init(I, 10, 10);
      }
      std::stringstream title;
      title << "Learning cube on image: " << imageName[i];
      // title << "Learning cube on image: " << g.getFrameIndex();
      vpDisplay::setTitle(I, title.str().c_str());

      vpDisplay::display(I);

      //! [Set tracker pose]
      tracker.setPose(I, initPoseTab[i]);
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
      std::cout<<imageName[i]<<" learned !"<<std::endl;
    }
    // g.close();
    //Tracker 2
    for (int i = 0; i < 4; i++) {
      vpImageIo::read(I, imageName2[i]);
      // if(i==0 || !g.end())
      //   g.acquire(I);
      // else
      //   std::cout<<"Missing training image !"<<std::endl;

      // vpArray2D<double>::loadYAML(cMoName[i],initPoseTab[i]);
      
      if (i == 0) {
        display.init(I, 10, 10);
      }
      std::stringstream title;
      title << "Learning cube on image: " << imageName2[i];
      // title << "Learning cube on image: " << g.getFrameIndex();
      vpDisplay::setTitle(I, title.str().c_str());

      vpDisplay::display(I);

      //! [Set tracker pose]
      tracker2.setPose(I, initPoseTab2[i]);
      //! [Set tracker pose]

      // std::cout<<"Refine"<<std::endl;
      //! [Refine pose]
      tracker2.track(I);
      //! [Refine pose]

      // std::cout<<"Display"<<std::endl;
      //! [Display tracker pose]
      tracker2.getPose(cMo2);
      tracker2.display(I, cMo2, cam, vpColor::red);
      //! [Display tracker pose]

      // std::cout<<"Learn"<<std::endl;
      //! [Learn cube call]
      learnCube(I, tracker2, keypoint_learning2, i);
      //! [Learn cube call]

      vpDisplay::displayText(I, 10, 10, "Learning step: keypoints are detected on visible cube faces", vpColor::red);
      if (i < 2) {
        vpDisplay::displayText(I, 30, 10, "Click to continue the learning...", vpColor::red);
      } else {
        vpDisplay::displayText(I, 30, 10, "Click to continue with the detection...", vpColor::red);
      }

      vpDisplay::flush(I);
      // vpDisplay::getClick(I, true);
      std::cout<<imageName2[i]<<" learned !"<<std::endl;
    }

    //! [Save learning data]
    keypoint_learning.saveLearningData("book_learning_data.bin", true); //BUG ? chargement des data bloqué si on enregistre pas les images
    keypoint_learning2.saveLearningData("book2_learning_data.bin", true);
    //! [Save learning data]
    // std::cout<<"Saved"<<std::endl;
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

    vpKeyPoint keypoint_detection2("ORB", "ORB", "BruteForce-Hamming");
    keypoint_detection2.setDetectorParameter("ORB", "nLevels", 1);
    //! [Init keypoint detection]
    // std::cout<<"Load"<<std::endl;
    //! [Load teabox learning data]
    keypoint_detection.loadLearningData("book_learning_data.bin", true);
    keypoint_detection2.loadLearningData("book2_learning_data.bin", true);
    //! [Load teabox learning data]
    // std::cout<<"Loaded"<<std::endl;

    //! [Create image matching]
    vpImage<unsigned char> IMatching;
    keypoint_detection.createImageMatching(I, IMatching);
    //! [Create image matching]

    videoname = "data_test_duo2/%03d.jpg";
    // std::cout<<"Reading : "<<videoname<<std::endl;
    vpVideoReader g;
    g.setFileName(videoname);
    g.open(I);

    // std::cout<<"Reading : "<<videoname<<std::endl;
    // std::string testName[] = {"train_image_000.png", "train_image_002.png", "train_image_003.png", "train_image_004.png", "train_image_005.png"};
    // vpImageIo::read(I, testName[0]);

#if defined VISP_HAVE_X11
    vpDisplayX display2;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display2;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display2;
#else
    vpDisplayOpenCV display2;
#endif
    display2.init(IMatching, 50, 50, "Display matching between learned and current images");
    vpDisplay::setTitle(I, "Cube detection and localization");

///// TEST ////
// vpRect* rect_roi = NULL;
// if(rect_roi==NULL)
//   rect_roi = new vpRect();
// double time=0, nb=0;
    display.init(I, 10, 10);
///////////////

    double error;
    bool click_done = false;

    while (!g.end()) {
    // for(unsigned int i = 1; i< 5; i++) {
    //   vpImageIo::read(I, testName[i]);
      // for(unsigned int i=0; i<20;i++)
      // {
      //   if(!g.end())
      //     g.acquire(I);
      //   else
      //     break;
      // }
      g.acquire(I);
      vpDisplay::display(I);

      //! [Insert image matching]
      keypoint_detection2.insertImageMatching(I, IMatching);
      //! [Insert image matching]

      vpDisplay::display(IMatching);
      vpDisplay::displayText(I, 10, 10, "Detection and localization in process...", vpColor::red);

      double elapsedTime;
      // ! [Matching and pose estimation]

      //Tracker 1
      // if (keypoint_detection.matchPoint(I, cam, cMo, error, elapsedTime,NULL,*rect_roi)) {
       if (keypoint_detection.matchPoint(I, cam, cMo, error, elapsedTime)) {
        //! [Matching and pose estimation]
        // std::cout<<"Match !"<<std::endl;

        //! [Display]
        // tracker.display(I, cMo, cam, vpColor::red, 2);
        vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 3);
        //! [Display]

        // keypoint_detection.displayMatching(I, IMatching);

        //! [Get RANSAC inliers outliers]
        std::vector<vpImagePoint> ransacInliers = keypoint_detection.getRansacInliers();
        std::vector<vpImagePoint> ransacOutliers = keypoint_detection.getRansacOutliers();
        //! [Get RANSAC inliers outliers]

        //! [Display RANSAC inliers]
        for (std::vector<vpImagePoint>::const_iterator it = ransacInliers.begin(); it != ransacInliers.end(); ++it) {
          vpDisplay::displayCircle(I, *it, 4, vpColor::green);
          vpImagePoint imPt(*it);
          imPt.set_u(imPt.get_u() + I.getWidth());
          imPt.set_v(imPt.get_v() + I.getHeight());
          vpDisplay::displayCircle(IMatching, imPt, 4, vpColor::green);
        }
        //! [Display RANSAC inliers]

        //! [Display RANSAC outliers]
        for (std::vector<vpImagePoint>::const_iterator it = ransacOutliers.begin(); it != ransacOutliers.end(); ++it) {
          vpDisplay::displayCircle(I, *it, 4, vpColor::red);
          vpImagePoint imPt(*it);
          imPt.set_u(imPt.get_u() + I.getWidth());
          imPt.set_v(imPt.get_v() + I.getHeight());
          vpDisplay::displayCircle(IMatching, imPt, 4, vpColor::red);
        }
        //! [Display RANSAC outliers]

        //! [Display image matching]
        // keypoint_detection.displayMatching(I, IMatching);
        //! [Display image matching]
      }
      else
      {
        std::cout<<"Object Lost in frame : "<<g.getFrameIndex()<<std::endl;
      }

      //Tracker 2
      if (keypoint_detection2.matchPoint(I, cam, cMo2, error, elapsedTime)) {
        //! [Matching and pose estimation]
        // std::cout<<"Match !"<<std::endl;

        //! [Display]
        // tracker.display(I, cMo, cam, vpColor::red, 2);
        vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 3);
        //! [Display]

        keypoint_detection2.displayMatching(I, IMatching);

        //! [Get RANSAC inliers outliers]
        std::vector<vpImagePoint> ransacInliers = keypoint_detection2.getRansacInliers();
        std::vector<vpImagePoint> ransacOutliers = keypoint_detection2.getRansacOutliers();
        //! [Get RANSAC inliers outliers]

        //! [Display RANSAC inliers]
        for (std::vector<vpImagePoint>::const_iterator it = ransacInliers.begin(); it != ransacInliers.end(); ++it) {
          vpDisplay::displayCircle(I, *it, 4, vpColor::green);
          vpImagePoint imPt(*it);
          imPt.set_u(imPt.get_u() + I.getWidth());
          imPt.set_v(imPt.get_v() + I.getHeight());
          vpDisplay::displayCircle(IMatching, imPt, 4, vpColor::green);
        }
        //! [Display RANSAC inliers]

        //! [Display RANSAC outliers]
        for (std::vector<vpImagePoint>::const_iterator it = ransacOutliers.begin(); it != ransacOutliers.end(); ++it) {
          vpDisplay::displayCircle(I, *it, 4, vpColor::red);
          vpImagePoint imPt(*it);
          imPt.set_u(imPt.get_u() + I.getWidth());
          imPt.set_v(imPt.get_v() + I.getHeight());
          vpDisplay::displayCircle(IMatching, imPt, 4, vpColor::red);
        }
        //! [Display RANSAC outliers]

        //! [Display image matching]
        keypoint_detection.displayMatching(I, IMatching);
        //! [Display image matching]
      }
      else
      {
        std::cout<<"Object 2 Lost in frame : "<<g.getFrameIndex()<<std::endl;
      }


      vpDisplay::flush(I);
      vpDisplay::displayText(IMatching, 30, 10, "A click to exit.", vpColor::red);
      vpDisplay::flush(IMatching);
      if (vpDisplay::getClick(I, false)) {
        click_done = true;
        break;
      }
      if (vpDisplay::getClick(IMatching, false)) {
        click_done = true;
        break;
      }
      // vpDisplay::getClick(I, true);
      // vpDisplay::getClick(IMatching, true);
    }

    // std::cout<<"Moyenne temps execution : "<<time/nb<<std::endl;
    // delete rect_roi;

    if (!click_done)
      vpDisplay::getClick(IMatching);

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
