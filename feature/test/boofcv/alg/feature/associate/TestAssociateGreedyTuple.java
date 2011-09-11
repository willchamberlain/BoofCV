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

package boofcv.alg.feature.associate;

import boofcv.struct.FastQueue;
import boofcv.struct.feature.TupleDescQueue;
import boofcv.struct.feature.TupleDesc_F64;
import org.junit.Test;

import static org.junit.Assert.assertEquals;


/**
 * @author Peter Abeles
 */
public class TestAssociateGreedyTuple {

	ScoreAssociateTuple score = new ScoreAssociateEuclidean();

	@Test
	public void basic() {
		FastQueue<TupleDesc_F64> a = createData(1,2,3,4);
		FastQueue<TupleDesc_F64> b = createData(3,4,1,40);

		int pairs[] = new int[4];

		AssociateGreedyTuple.basic(a,b,score,0.5,pairs);

		assertEquals(2,pairs[0]);
		assertEquals(-1,pairs[1]);
		assertEquals(0,pairs[2]);
		assertEquals(1,pairs[3]);
	}

	@Test
	public void fitIsError() {
		FastQueue<TupleDesc_F64> a = createData(1,2,3,4);
		FastQueue<TupleDesc_F64> b = createData(3,4,1,40);

		int pairs[] = new int[4];
		double fitScore[] = new double[4];

		AssociateGreedyTuple.fitIsError(a,b,score,pairs,fitScore);

		assertEquals(2,pairs[0]);
		assertEquals(0,pairs[1]);
		assertEquals(0,pairs[2]);
		assertEquals(1,pairs[3]);

		assertEquals(0,fitScore[0],1e-5);
		assertEquals(1,fitScore[1],1e-5);
		assertEquals(0,fitScore[2],1e-5);
		assertEquals(0,fitScore[3],1e-5);
	}

	@Test
	public void totalCloseMatches() {
		FastQueue<TupleDesc_F64> a = createData(1,2,3,4);
		FastQueue<TupleDesc_F64> b = createData(3,4,1,40);

		int pairs[] = new int[4];
		double fitScore[] = new double[4];
		double workBuffer[] = new double[4];

		AssociateGreedyTuple.totalCloseMatches(a,b,score,2,workBuffer,pairs,fitScore);

		assertEquals(2,pairs[0]);
		assertEquals(0,pairs[1]);
		assertEquals(0,pairs[2]);
		assertEquals(1,pairs[3]);

		assertEquals(1,fitScore[0],1e-5);
		assertEquals(3,fitScore[1],1e-5);
		assertEquals(1,fitScore[2],1e-5);
		assertEquals(1,fitScore[3],1e-5);
	}

	@Test
	public void forwardBackwards() {
		FastQueue<TupleDesc_F64> a = createData(1,2,3,4);
		FastQueue<TupleDesc_F64> b = createData(3,4,1,40);

		int pairs[] = new int[4];
		double workBuffer[] = new double[4*4];
		double fitScore[] = new double[4];

		AssociateGreedyTuple.forwardBackwards(a,b,score,workBuffer,pairs,fitScore);

		assertEquals(2,pairs[0]);
		assertEquals(-1,pairs[1]);
		assertEquals(0,pairs[2]);
		assertEquals(1,pairs[3]);

		assertEquals(0,fitScore[0],1e-5);
		assertEquals(0,fitScore[2],1e-5);
		assertEquals(0,fitScore[3],1e-5);
	}

	private FastQueue<TupleDesc_F64> createData( double ...values )
	{
		FastQueue<TupleDesc_F64> ret = new TupleDescQueue(1, true);

		for( int i = 0; i < values.length; i++ ) {
			ret.pop().set(values[i]);
		}

		return ret;
	}
}
