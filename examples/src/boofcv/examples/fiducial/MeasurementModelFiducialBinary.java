/*
 * Copyright (c) 2011-2016, Peter Abeles. All Rights Reserved.
 *
 * This file is part of BoofCV (http://boofcv.org).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package boofcv.examples.fiducial;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.alg.distort.LensDistortionNarrowFOV;
import boofcv.alg.distort.radtan.LensDistortionRadialTangential;
import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.gui.fiducial.VisualizeFiducial;
import boofcv.gui.image.ShowImages;
import boofcv.io.UtilIO;
import boofcv.io.calibration.CalibrationIO;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.calib.CameraPinholeRadial;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.ImageType;
import georegression.struct.point.Point2D_F64;
import georegression.struct.se.Se3_F64;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.List;

/**
 * Detects square binary fiducials inside an image, writes out there pose, and visualizes a virtual flat cube
 * above them in the input image.
 *
 * @author Peter Abeles
 */
public class MeasurementModelFiducialBinary {
	public static void main(String[] args) {

		//String directory = UtilIO.path("/mnt/nixbig/ownCloud/project_code/measurement_model_boofcv/measurement_model_boofcv")	; // pathExample("fiducial/binary");
		String directory = "/mnt/nixbig/ownCloud/project_code/measurement_model_boofcv/measurement_model_boofcv" ;   // pathExample("fiducial/binary");

		// load the lens distortion parameters and the input image
		CameraPinholeRadial param = CalibrationIO.load(new File("/mnt/nixbig/downloads/boofcv_will/BoofCV_cp/examples/src/boofcv/examples/fiducial/camera_intrinsic.yaml"));		
		LensDistortionNarrowFOV lensDistortion = new LensDistortionRadialTangential(param);

		List<BufferedImage> inputs = UtilImageIO.loadImages(directory  , ".*\\.png" ) ;
		// BufferedImage input = UtilImageIO.loadImage(directory , "image0000.jpg");
//		BufferedImage input = UtilImageIO.loadImage(directory , "image0001.jpg");
//		BufferedImage input = UtilImageIO.loadImage(directory , "image0002.jpg");
		for(BufferedImage input : inputs ) {
			GrayF32 original = ConvertBufferedImage.convertFrom(input,true, ImageType.single(GrayF32.class));

			// Detect the fiducial
			FiducialDetector<GrayF32> detector = FactoryFiducial.squareBinary(
					new ConfigFiducialBinary(0.1), ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10), GrayF32.class);
	//				new ConfigFiducialBinary(0.1), ConfigThreshold.fixed(100),GrayF32.class);

			detector.setLensDistortion(lensDistortion);
			detector.detect(original);

			// // print the results
			// Graphics2D g2 = input.createGraphics();
			// Se3_F64 targetToSensor = new Se3_F64();
			// Point2D_F64 locationPixel = new Point2D_F64();
			// for (int i = 0; i < detector.totalFound(); i++) {
			// 	detector.getImageLocation(i, locationPixel);

			// 	if( detector.hasUniqueID() )
			// 		System.out.println("Target ID = "+detector.getId(i));
			// 	if( detector.hasMessage() )
			// 		System.out.println("Message   = "+detector.getMessage(i));
			// 	System.out.println("2D Image Location = "+locationPixel);

			// 	if( detector.is3D() ) {
			// 		detector.getFiducialToCamera(i, targetToSensor);
			// 		System.out.println("3D Location:");
			// 		System.out.println(targetToSensor);
			// 		VisualizeFiducial.drawCube(targetToSensor, param, detector.getWidth(i), 3, g2);
			// 		VisualizeFiducial.drawLabelCenter(targetToSensor, param, "" + detector.getId(i), g2);
			// 	} else {
			// 		VisualizeFiducial.drawLabel(locationPixel, "" + detector.getId(i), g2);
			// 	}
			// }

			// ShowImages.showWindow(input,"Fiducials",true);
			}
	}
}
