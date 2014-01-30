/*
 * Copyright (c) 2011-2014, Peter Abeles. All Rights Reserved.
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

package boofcv.alg.segmentation.fh04;

import boofcv.struct.image.ImageUInt8;
import boofcv.struct.image.MultiSpectral;
import org.ddogleg.struct.FastQueue;

import static boofcv.alg.segmentation.fh04.SegmentFelzenHutten04.Edge;

/**
 * Computes edge weight as the absolute value of the different in pixel value.
 * A 4-connect neighborhood is considered.
 *
 * @author Peter Abeles
 */
// TODO create code generator
public class ComputeEdgeWeights8_MsU8 implements ComputeEdgeWeights<MultiSpectral<ImageUInt8>> {


	int pixelColor[];
	int numBands;

	public ComputeEdgeWeights8_MsU8( int numBands ) {
		this.numBands = numBands;
		pixelColor = new int[numBands];
	}

	@Override
	public void process(MultiSpectral<ImageUInt8> input,
						int outputStartIndex , int outputStride ,
						FastQueue<Edge> edges) {

		edges.reset();
		int w = input.width-1;
		int h = input.height-1;

		// First consider the inner pixels
		for( int y = 0; y < h; y++ ) {
			int indexSrc = input.startIndex + y*input.stride + 1;
			int indexDst = outputStartIndex + y*outputStride + 1;

			for( int x = 1; x < w; x++ , indexSrc++ , indexDst++ ) {

				int weight1=0,weight2=0,weight3=0,weight4=0;

				for( int i = 0; i < numBands; i++ ) {
					ImageUInt8 band = input.getBand(i);

					int color0 = band.data[indexSrc] & 0xFF;                // (x,y)
					int color1 = band.data[indexSrc+1] & 0xFF;              // (x+1,y)
					int color2 = band.data[indexSrc+input.stride] & 0xFF;   // (x,y+1)
					int color3 = band.data[indexSrc+1+input.stride] & 0xFF; // (x+1,y+1)
					int color4 = band.data[indexSrc-1+input.stride] & 0xFF; // (x-1,y+1)

					int diff1 = color0-color1;
					int diff2 = color0-color2;
					int diff3 = color0-color3;
					int diff4 = color0-color4;

					weight1 += diff1*diff1;
					weight2 += diff2*diff2;
					weight3 += diff3*diff3;
					weight4 += diff4*diff4;
				}

				Edge e1 = edges.grow();
				Edge e2 = edges.grow();
				Edge e3 = edges.grow();
				Edge e4 = edges.grow();

				e1.sortValue = (float)Math.sqrt(weight1);
				e1.indexA = indexDst;
				e1.indexB = indexDst+1;

				e2.sortValue = (float)Math.sqrt(weight2);
				e2.indexA = indexDst;
				e2.indexB = indexDst+outputStride;

				e3.sortValue = (float)Math.sqrt(weight3);
				e3.indexA = indexDst;
				e3.indexB = indexDst+1+outputStride;

				e4.sortValue = (float)Math.sqrt(weight4);
				e4.indexA = indexDst;
				e4.indexB = indexDst-1+outputStride;
			}
		}

		// Handle border pixels

		for( int y = 0; y < h; y++ ) {
			checkAround(0,y,input,outputStartIndex,outputStride,edges);
			checkAround(w,y,input,outputStartIndex,outputStride,edges);
		}

		for( int x = 0; x < w; x++ ) {
			checkAround(x,h,input,outputStartIndex,outputStride,edges);
		}
	}

	private void checkAround( int x , int y ,
							  MultiSpectral<ImageUInt8> input ,
							  int outputStartIndex , int outputStride ,
							  FastQueue<Edge> edges )
	{
		int indexA = input.startIndex + y*input.stride + x;

		for( int i = 0; i < numBands; i++ ) {
			ImageUInt8 band = input.getBand(i);
			pixelColor[i] =  band.data[indexA] & 0xFF;
		}

		check(x+1,y , pixelColor,indexA,input,outputStartIndex,outputStride,edges);
		check(x  ,y+1,pixelColor,indexA,input,outputStartIndex,outputStride,edges);
		check(x+1,y+1,pixelColor,indexA,input,outputStartIndex,outputStride,edges);
		check(x-1,y+1,pixelColor,indexA,input,outputStartIndex,outputStride,edges);
	}

	private void check( int x , int y , int color0[] , int indexA,
						MultiSpectral<ImageUInt8> input ,
						int outputStartIndex , int outputStride ,
						FastQueue<Edge> edges ) {
		if( !input.isInBounds(x,y) )
			return;

		int indexSrc = input.startIndex + y*input.stride + x;
		int indexB   = outputStartIndex + y*outputStride + x;

		float weight = 0;

		for( int i = 0; i < numBands; i++ ) {
			ImageUInt8 band = input.getBand(i);

			int color = band.data[indexSrc] & 0xFF;
			int diff = color0[i]-color;
			weight += diff*diff;
		}

		Edge e1 = edges.grow();

		e1.sortValue = (float)Math.sqrt(weight);
		e1.indexA = indexA;
		e1.indexB = indexB;
	}
}