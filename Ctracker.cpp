#include "Ctracker.h"
using namespace cv;
using namespace std;

size_t CTrack::NextTrackID=0;
// ---------------------------------------------------------------------------
// ����������� �����.
// ��� ��������, ���� ���������� � ����� �� �����,
// ��� ����� � ���������� ������������ � �������� ���������.
// ---------------------------------------------------------------------------
CTrack::CTrack(Point2f pt, float dt, float Accel_noise_mag)
{
	track_id=NextTrackID;

	NextTrackID++;
	// ������ ���� ����� ���� ������ ��������,
	// ��� ������ �������� �������� �������, ��� ������ ���� ��������� �����.
	KF = new TKalmanFilter(pt,dt,Accel_noise_mag);
	// ����� �������� ���������� �����, � ������� ���� ������������ ��������� ���������� (������).
	prediction=pt;
	skipped_frames=0;
}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
CTrack::~CTrack()
{
	// ����������� ������ ��������.
	delete KF;
}

// ---------------------------------------------------------------------------
// ������. ���������� ���������� �������. �������, �������, ��������.
// ---------------------------------------------------------------------------
CTracker::CTracker(float _dt, float _Accel_noise_mag, double _dist_thres, int _maximum_allowed_skipped_frames,int _max_trace_length)
{
dt=_dt;
Accel_noise_mag=_Accel_noise_mag;
dist_thres=_dist_thres;
maximum_allowed_skipped_frames=_maximum_allowed_skipped_frames;
max_trace_length=_max_trace_length;
}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
void CTracker::Update(vector<Point2d>& detections, int& counter_in, int& counter_out)
{
	// -----------------------------------
	// ���� ������ ��� ���, �� ������ ��� ������ ����� �� �����
	// -----------------------------------
	if(tracks.size()==0)
	{
		// ���� ��� ��� �� ������ �����
		for(int i=0;i<detections.size();i++)
		{
			CTrack* tr=new CTrack(detections[i],dt,Accel_noise_mag);
			tracks.push_back(tr);
		}	
	}
	// -----------------------------------
	// ����� ����� ��� ���� � ����� ������
	// -----------------------------------
	int N=tracks.size();		// tracks
	int M=detections.size();	// points detected

	// ������� ���������� �� N-���� ����� �� M-���� �������.
	vector< vector<double> > Cost(N,vector<double>(M));
	vector<int> assignment; // ����������

	// -----------------------------------
	// ����� ��� ����, �������� ������� ����������
	// -----------------------------------
	double dist;
	for(int i=0;i<tracks.size();i++)
	{	
		// Point2d prediction=tracks[i]->prediction;
		// cout << prediction << endl;
		for(int j=0;j<detections.size();j++)
		{
			Point2d diff=(tracks[i]->prediction-detections[j]);
			dist=sqrtf(diff.x*diff.x+diff.y*diff.y);
			Cost[i][j]=dist;
		}
	}
	// -----------------------------------
	// ������ ������ � ����������� (����� � �������� �������)
	// -----------------------------------
	AssignmentProblemSolver APS;
	APS.Solve(Cost,assignment,AssignmentProblemSolver::optimal);

	// -----------------------------------
	// �������� assignment �� ��� � ������� �����������
	// -----------------------------------
	// �� ����������� �����
	vector<int> not_assigned_tracks;

	for(int i=0;i<assignment.size();i++)
	{
		if(assignment[i]!=-1)
		{
			
			if(Cost[i][assignment[i]]>dist_thres)
			{
				assignment[i]=-1;
				// �������� ������������� �����, � ����������� ������� ���������� ������,
				// ����� ���������� ����������� ������ �������� ��������� ��������, ���� ���������.
				not_assigned_tracks.push_back(i);
			}
		}
		else
		{			
			// ���� ����� �� �������� ������, �� ����������� ������� ���������� ������.
			tracks[i]->skipped_frames++;
		}

	} 
	// -----------------------------------
	// �������� ������������� �������
	// -----------------------------------
	vector<int> not_assigned_detections;
	vector<int>::iterator it;
	for(int i=0;i<detections.size();i++)
	{
		it=find(assignment.begin(), assignment.end(), i);
		if(it==assignment.end())
		{
			not_assigned_detections.push_back(i);
		}
	}

	// -----------------------------------
	// � �������� ��� ��� ����� �����
	// -----------------------------------
	if(not_assigned_detections.size()!=0)
	{
		for(int i=0;i<not_assigned_detections.size();i++)
		{
			CTrack* tr=new CTrack(detections[not_assigned_detections[i]],dt,Accel_noise_mag);
			tracks.push_back(tr);
		}	
	}

	// �������� ��������� ��������

	for(int i=0;i<assignment.size();i++)
	{
		// ���� ���� ���������� ������ ������ ����, �� ��������� ������� �����������.

		tracks[i]->KF->GetPrediction();

		if(assignment[i]!=-1) // ���� ���������� ���� �� �������� �� ����
		{
			tracks[i]->skipped_frames=0;
			tracks[i]->prediction=tracks[i]->KF->Update(detections[assignment[i]],1);
		}else				  // ���� ���, �� ���������� ��������������
		{
			tracks[i]->prediction=tracks[i]->KF->Update(Point2f(0,0),0);	
		}
		
		if(tracks[i]->trace.size()>max_trace_length)
		{
			tracks[i]->trace.erase(tracks[i]->trace.begin(),tracks[i]->trace.end()-max_trace_length);
		}

		tracks[i]->trace.push_back(tracks[i]->prediction);
		tracks[i]->KF->LastResult=tracks[i]->prediction;
	}
	
	// -----------------------------------
	// ���� ���� ����� �� �������� ��������, �������
	// Something in this loop do..
	// -----------------------------------
           
	for(unsigned int i=0;i<tracks.size();i++)
	{
		if(tracks[i]->skipped_frames>maximum_allowed_skipped_frames)
		{
			//counting people
			if(tracks[i]->trace.size()>15)
			  
			{
			if(tracks[i]->trace.back().y<200 && tracks[i]->trace.front().y>230)
			  counter_out++;
			if(tracks[i]->trace.back().y>230 && tracks[i]->trace.front().y<200)
			  counter_in++;
			}
			//end of counting
			delete tracks[i];
			tracks.erase(tracks.begin()+i);
			assignment.erase(assignment.begin()+i);
			i--;
			
			
		}
	}

}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
CTracker::~CTracker(void)
{
	for(int i=0;i<tracks.size();i++)
	{
	delete tracks[i];
	}
	tracks.clear();
}
