package org.boofcv.android.sfm;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.LinearLayout;
import android.widget.Spinner;

import org.boofcv.android.DemoBitmapCamera2Activity;
import org.boofcv.android.DemoProcessingAbstract;
import org.boofcv.android.R;
import org.ddogleg.struct.FastQueue;

import java.util.List;

import boofcv.abst.feature.detect.interest.ConfigGeneralDetector;
import boofcv.abst.sfm.AccessPointTracks;
import boofcv.abst.sfm.d2.ImageMotion2D;
import boofcv.abst.sfm.d2.PlToGrayMotion2D;
import boofcv.abst.tracker.PointTracker;
import boofcv.alg.sfm.d2.StitchingFromMotion2D;
import boofcv.factory.sfm.FactoryMotion2D;
import boofcv.factory.tracker.FactoryPointTracker;
import boofcv.struct.image.GrayS16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.ImageType;
import boofcv.struct.image.Planar;
import georegression.struct.affine.Affine2D_F64;
import georegression.struct.homography.Homography2D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.transform.homography.HomographyPointOps_F64;

/**
 * Displays image stabilization.
 *
 * @author Peter Abeles
 */
public class StabilizeDisplayActivity extends DemoBitmapCamera2Activity
implements CompoundButton.OnCheckedChangeListener
{

	Paint paintBorder=new Paint();
	Paint paintInlier=new Paint();
	Paint paintOutlier=new Paint();

	boolean showFeatures;
	boolean resetRequested;

	Spinner spinnerView;

	public StabilizeDisplayActivity() {
		super(Resolution.MEDIUM);
		super.changeResolutionOnSlow = true;
	}

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		paintInlier.setColor(Color.GREEN);
		paintInlier.setStyle(Paint.Style.FILL);
		paintOutlier.setColor(Color.RED);
		paintOutlier.setStyle(Paint.Style.FILL);
		paintBorder.setColor(Color.CYAN);
		paintBorder.setStyle(Paint.Style.STROKE);

		resetRequested = false;

		LayoutInflater inflater = getLayoutInflater();
		LinearLayout controls = (LinearLayout)inflater.inflate(R.layout.stabilization_controls,null);

		spinnerView = controls.findViewById(R.id.spinner_algs);
		ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(this,
				R.array.trackers_stabilize, android.R.layout.simple_spinner_item);
		adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
		spinnerView.setAdapter(adapter);
		spinnerView.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
			@Override
			public void onItemSelected(AdapterView<?> adapterView, View view, int i, long l) {
				createNewProcessor();
			}

			@Override
			public void onNothingSelected(AdapterView<?> adapterView) {}
		});

		CheckBox seek = controls.findViewById(R.id.check_features);
		seek.setOnCheckedChangeListener(this);

		setControls(controls);
	}

	@Override
	public void createNewProcessor() {
		StitchingFromMotion2D<Planar<GrayU8>,Affine2D_F64> distortAlg =
				createStabilization(spinnerView.getSelectedItemPosition());
		setProcessing(new PointProcessing(distortAlg));
	}

	public void resetPressed(View view ) {
		resetRequested = true;
	}

	static StitchingFromMotion2D<Planar<GrayU8>,Affine2D_F64> createStabilization( int which ) {

		PointTracker<GrayU8> tracker;

		if( which == 0 ) {
			ConfigGeneralDetector config = new ConfigGeneralDetector();
			config.maxFeatures = 200;
			config.threshold = 10;
			config.radius = 2;

			tracker = FactoryPointTracker.
					klt(new int[]{1, 2,4,8}, config, 3, GrayU8.class, GrayS16.class);
		} else {
			tracker = FactoryPointTracker.dda_FH_SURF_Fast(null,null,null,GrayU8.class);
		}


        ImageMotion2D<GrayU8,Affine2D_F64> motion2D =
                FactoryMotion2D.createMotion2D(100,1.5,2,40,0.5,0.6,false,tracker,new Affine2D_F64());

        // wrap it so it output color images while estimating motion from gray
        ImageMotion2D<Planar<GrayU8>,Affine2D_F64> motion2DColor =
                new PlToGrayMotion2D<>(motion2D, GrayU8.class);

        // This fuses the images together
        StitchingFromMotion2D<Planar<GrayU8>,Affine2D_F64>
                stabilize = FactoryMotion2D.createVideoStitch(0.01, motion2DColor,ImageType.pl(3,GrayU8.class));

        return stabilize;

//		ImageMotion2D<GrayU8,Affine2D_F64> motion = FactoryMotion2D.createMotion2D(100, 1.5, 2, 40,
//				0.5, 0.6, false, tracker, new Affine2D_F64());
//
//		return FactoryMotion2D.createVideoStitch(0.01,motion, ImageType.single(GrayU8.class));
	}

	@Override
	public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
		showFeatures = b;
	}

	protected class PointProcessing extends DemoProcessingAbstract<Planar<GrayU8>> {
		StitchingFromMotion2D<Planar<GrayU8>,Affine2D_F64> alg;
		Homography2D_F64 imageToDistorted = new Homography2D_F64();
		Homography2D_F64 distortedToImage = new Homography2D_F64();


		StitchingFromMotion2D.Corners corners = new StitchingFromMotion2D.Corners();
		Point2D_F64 distPt = new Point2D_F64();

		FastQueue<Point2D_F64> inliersGui = new FastQueue<Point2D_F64>(Point2D_F64.class,true);
		FastQueue<Point2D_F64> outliersGui = new FastQueue<Point2D_F64>(Point2D_F64.class,true);

		float radius;

		public PointProcessing( StitchingFromMotion2D<Planar<GrayU8>,Affine2D_F64> alg  ) {
			super(ImageType.pl(3,GrayU8.class));
			this.alg = alg;
		}

		@Override
		public void initialize(int imageWidth, int imageHeight, int sensorOrientation) {
			alg.configure(imageWidth,imageHeight,null);

			float density = cameraToDisplayDensity;
			radius = 3*density;
			paintBorder.setStrokeWidth(5*density);
		}

		@Override
		public void onDraw(Canvas canvas, Matrix imageToView) {
			drawBitmap(canvas,imageToView);

			if( !showFeatures )
				return;

			canvas.concat(imageToView);
			synchronized (lockGui) {
				Point2D_F64 p0 = corners.p0;
				Point2D_F64 p1 = corners.p1;
				Point2D_F64 p2 = corners.p2;
				Point2D_F64 p3 = corners.p3;

				canvas.drawLine((int) p0.x, (int) p0.y, (int) p1.x, (int) p1.y, paintBorder);
				canvas.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y, paintBorder);
				canvas.drawLine((int) p2.x, (int) p2.y, (int) p3.x, (int) p3.y, paintBorder);
				canvas.drawLine((int) p3.x, (int) p3.y, (int) p0.x, (int) p0.y, paintBorder);

				for (int i = 0; i < inliersGui.size; i++) {
					Point2D_F64 p = inliersGui.get(i);
					canvas.drawCircle((float) p.x, (float) p.y, radius, paintInlier);
				}
				for (int i = 0; i < outliersGui.size; i++) {
					Point2D_F64 p = outliersGui.get(i);
					canvas.drawCircle((float) p.x, (float) p.y, radius, paintOutlier);
				}
			}
		}

		@Override
		public void process(Planar<GrayU8> gray) {
			if( !resetRequested && alg.process(gray) ) {
				convertToBitmapDisplay(alg.getStitchedImage());
				synchronized ( lockGui ) {
					alg.getImageCorners(gray.width,gray.height,corners);

					ImageMotion2D<?,?> motion = alg.getMotion();
					if( showFeatures && (motion instanceof AccessPointTracks) ) {
						AccessPointTracks access = (AccessPointTracks)motion;

						alg.getWorldToCurr(imageToDistorted);
						imageToDistorted.invert(distortedToImage);
						inliersGui.reset();outliersGui.reset();
						List<Point2D_F64> points = access.getAllTracks();
						for( int i = 0; i < points.size(); i++ ) {
							HomographyPointOps_F64.transform(distortedToImage,points.get(i),distPt);

							if( access.isInlier(i) ) {
								inliersGui.grow().set(distPt.x,distPt.y);
							} else {
								outliersGui.grow().set(distPt.x,distPt.y);
							}
						}
					}
				}
			} else {
				resetRequested = false;
				alg.reset();
			}
		}

	}
}