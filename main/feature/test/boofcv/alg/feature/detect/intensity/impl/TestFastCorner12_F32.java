/*
 * Copyright (c) 2011, Peter Abeles. All Rights Reserved.
 *
 * This file is part of BoofCV (http://www.boofcv.org).
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

package boofcv.alg.feature.detect.intensity.impl;

import boofcv.struct.image.ImageBase;
import boofcv.struct.image.ImageFloat32;
import georegression.struct.point.Point2D_I16;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * @author Peter Abeles
 */
public class TestFastCorner12_F32 {
	private int[] offsets;

	int width = 7;
	int height = 9;
	float centerVal = 100;

	public TestFastCorner12_F32() {

	}

	private void setOffsets(ImageBase img) {
		int stride = img.stride;

		int center = 3 * stride + 3;

		offsets = new int[16];
		offsets[0] = center - 3;
		offsets[1] = center - 3 - stride;
		offsets[2] = center - 2 - 2 * stride;
		offsets[3] = center - 1 - 3 * stride;
		offsets[4] = center - 3 * stride;
		offsets[5] = center + 1 - 3 * stride;
		offsets[6] = center + 2 - 2 * stride;
		offsets[7] = center + 3 - stride;
		offsets[8] = center + 3;
		offsets[9] = center + 3 + stride;
		offsets[10] = center + 2 + 2 * stride;
		offsets[11] = center + 1 + 3 * stride;
		offsets[12] = center + 3 * stride;
		offsets[13] = center - 1 + 3 * stride;
		offsets[14] = center - 2 + 2 * stride;
		offsets[15] = center - 3 + stride;
	}

	/**
	 * Create a set of synthetic images and see if it correctly identifies them as corners.
	 */
	@Test
	public void testPositive() {
		FastCorner12_F32 corner = new FastCorner12_F32(20, 12);
		ImageFloat32 img = new ImageFloat32(width, height);

		for( int subImage = 0; subImage < 2; subImage++ ) {
			// see if it can handle a sub image correctly
			if( subImage == 1 ) {
				ImageFloat32 tmp = new ImageFloat32( width+4,height+5);
				img = tmp.subimage(2,3,2+width,3+height);
			}
			setOffsets(img);

			// pixels in circle are lower than threshold
			for (int i = 0; i < 15; i++) {
				setSynthetic(img, i, 12, (centerVal - 50));

				corner.process(img);

				assertEquals(1, countNonZero(corner.getIntensity()));
				assertEquals(1, corner.getCandidates().size());
				// feature intensity should be positive and more than zero
				Point2D_I16 c = corner.getCandidates().get(0);
				float intensity = corner.getIntensity().get(c.x,c.y);
				assertTrue(intensity > 0);
			}

			// pixels in circle are higher than threshold
			for (int i = 0; i < 15; i++) {
				setSynthetic(img, i, 12, (centerVal + 50));

				corner.process(img);

				assertEquals(1, countNonZero(corner.getIntensity()));
				assertEquals(1, corner.getCandidates().size());
				Point2D_I16 c = corner.getCandidates().get(0);
				assertTrue(corner.getIntensity().get(c.x,c.y) > 0 );
			}

			// longer than needed
			for (int i = 0; i < 15; i++) {
				setSynthetic(img, i, 13, (centerVal + 50));

				corner.process(img);

				assertEquals(1, countNonZero(corner.getIntensity()));
				assertEquals(1, corner.getCandidates().size());
				Point2D_I16 c = corner.getCandidates().get(0);
				assertTrue(corner.getIntensity().get(c.x,c.y) > 0 );
			}
		}

	}

	private static int countNonZero(ImageFloat32 img) {
		float[] data = img.data;

		int ret = 0;
		for (float aData : data) {
			if (aData != 0)
				ret++;
		}
		return ret;
	}

	/**
	 * See if it classifies a circle that is too short
	 */
	@Test
	public void testNegativeShort() {
		FastCorner12_F32 corner = new FastCorner12_F32(20, 12);
		ImageFloat32 img = new ImageFloat32(width, height);
		setOffsets(img);

		for (int i = 0; i < 15; i++) {
			setSynthetic(img, i, 11, (centerVal + 50));

			corner.process(img);

			assertEquals(0, countNonZero(corner.getIntensity()));
			assertEquals(0, corner.getCandidates().size());
		}
	}

	/**
	 * Both pixels that are too high and low, but exceed the threshold are mixed
	 */
	@Test
	public void testNegativeMixed() {
		FastCorner12_F32 corner = new FastCorner12_F32( 20, 12);
		ImageFloat32 img = new ImageFloat32(width, height);
		setOffsets(img);

		for (int i = 0; i < 15; i++) {
			setSynthetic(img, i, 12, (centerVal + 50));

			img.data[offsets[(i + 7) % offsets.length]] = centerVal - 50;

			corner.process(img);

			assertEquals(0, countNonZero(corner.getIntensity()));
		}
	}

	private void setSynthetic(ImageFloat32 img, int start, int length, float outerVal) {
		float data[] = img.data;

		int endA = start + length;
		int endB;

		if (endA > offsets.length) {
			endB = endA - offsets.length;
			endA = offsets.length;
		} else {
			endB = 0;
		}

		for (int i = 0; i < width; i++) {
			for (int j = 0; j < width; j++) {
				img.set(i, j, centerVal);
			}
		}

		for (int i = start; i < endA; i++) {
			data[img.startIndex+offsets[i]] = outerVal;
		}

		for (int i = 0; i < endB; i++) {
			data[img.startIndex+offsets[i]] = outerVal;
		}
	}

}