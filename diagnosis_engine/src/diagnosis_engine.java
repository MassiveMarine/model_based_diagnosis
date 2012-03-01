import com.google.common.base.Preconditions;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import java.util.*;
import java.io.*;
import java.text.*;
import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import java.lang.System;
import org.ros.message.diagnosis_msgs.Diagnosis;
import org.ros.message.diagnosis_msgs.Observations;
import org.ros.message.diagnostic_msgs.DiagnosticArray;

import hittingsetalg.*;
import theoremprover.*;
import utils.*;
import dfengine.*;

/**
 * This is a diagnosic_engine class of the Model Based Diagnostics. It assumes an
 * external roscore is already running.
 * 
 * @authors Safdar Zaman, Gerald Steinbauer. (szaman@ist.tugraz.at, steinbauer@ist.tugraz.at)
 */

class IllegalUserInput extends Exception {

    public IllegalUserInput(String msg) {
        super(msg);
    }

}

public class diagnosis_engine extends Thread implements NodeMain {
 
  private String PROP;
  private String SD;
  private String OBS;
  private String AB;
  private String NAB;
  private String neg_prefix;
  
  private Node node;
  private ArrayList<String> msg_list = new ArrayList<String>();
  private boolean processObs;
  private boolean threadRunning;
 	private Publisher<org.ros.message.diagnosis_msgs.Diagnosis> publisher;
  private Publisher<org.ros.message.diagnostic_msgs.DiagnosticArray> d_pub;

  private Calendar now = Calendar.getInstance();

	public diagnosis_engine()
  {
       super("FetchResult"+1);
       AB = "AB";
       NAB = "NAB";
       neg_prefix = "not_";
			 processObs = false;
			 threadRunning =  false;
       
  }



  @Override
  public void onStart(Node node){ 
    try{
        main_engine(node);
        }catch(Exception e){}
  }
  public void main_engine(Node node) throws ParseError,
        IllegalUserInput{
  try {
      this.node = node;
      String path=null;
      final Log log = node.getLog();
      
      System.out.println("Diagnosis Engine is up.......");
      publisher = node.newPublisher("/Diagnosis", "diagnosis_msgs/Diagnosis");
      d_pub = node.newPublisher("/diagnostics", "diagnostic_msgs/DiagnosticArray"); 

      node.newSubscriber("/Diagnostic_Model", "diagnosis_msgs/SystemDescription",
          new MessageListener<org.ros.message.diagnosis_msgs.SystemDescription>() {
            @Override
            public void onNewMessage(org.ros.message.diagnosis_msgs.SystemDescription sd_msg) {
            try { 
                 String[] rules = (String[]) sd_msg.rules.toArray(new String[0]);
                 String[] props = (String[]) sd_msg.props.toArray(new String[0]);
                 //System.out.println("Subscriber got up");
                 System.out.println("Time="+sd_msg.out_time+",AB="+sd_msg.AB+",NAB="+sd_msg.NAB+",Neg_Prefix="+sd_msg.neg_prefix+", No of  				 Rules:"+rules.length+", No of Props:"+props.length);
                  String s="";
                  for(int m=0; m<rules.length-1; m++)
                  {  s = s + rules[m] + ".\r\n";
                  }
                  s = s + rules[rules.length-1] + ".\r\n\r\n\r\n";
                  SD = s;
                 s="";
                 for(int m=0; m<props.length-1; m++)
                  {  s = s + props[m] + "\r\n";
                  }
								 s = s + props[props.length-1] + "\r\n\r\n\r\n";
                 PROP = s;
                 AB = sd_msg.AB;
                 NAB = sd_msg.NAB;
                 neg_prefix = sd_msg.neg_prefix;
                 System.out.println("AB="+AB+",NAB="+NAB+",Neg_Prefix="+neg_prefix+", Rules:"+SD+", Props:"+ PROP);
                 if(!threadRunning){
	                    start();
											threadRunning = true;
                     }
                 
                }catch(Exception e) 
                 {System.out.println("Error");System.out.println(e);
                  }
          
      }
      }
      );


node.newSubscriber("/Diagnostic_Observation", "diagnosis_msgs/Observations",
          new MessageListener<org.ros.message.diagnosis_msgs.Observations>() {
            @Override
            public void onNewMessage(org.ros.message.diagnosis_msgs.Observations msg) {
              //if(!processObs)
              {
							String[] obs_msg = (String[]) msg.obs.toArray(new String[0]);
               for(int m=0; m<obs_msg.length; m++)
                 { boolean found = false;
                   String s = obs_msg[m];
                   String ns = "@";
                   if(s.charAt(0)=='~')
                     { 
                       ns = s.substring(1);
                       s = neg_prefix + s.substring(1);
                     }
                    else
                       ns = neg_prefix + s;

                   //for(String st : msg_list)
                      //if(s.equals(st))
                     //for(int i=0;i<msg_list.size();i++)
                     if(msg_list.contains(s))
             					  {
               					  found = true;
               					  break;
              				   }
                            
                  if(!found)
         						{
                      //for(int j=0;j<msg_list.size();j++)
                         int k = msg_list.indexOf(ns);
            							if(k!=-1)
                						{
                              msg_list.set(k,s);
                              found = true;
                 							break;
                						}
                      
                      if(!found)
											 msg_list.add(s);
          				 } // if(!found)
                  /*for(int k=0;k<msg_list.size();k++)
                      System.out.println(","+msg_list.get(k).toString());
                  System.out.println("SIZE="+msg_list.size());*/
                
                } // for int m
             
            } // if(!processObs)
           } //onMessage   
          });

   
    } catch (Exception e) {
      if (node != null) {
        System.out.println(e);
        node.getLog().fatal(e);
        
      } else {
        e.printStackTrace();
      }
    }
        

  } //main


void find_diag()
    {   

    try{ 
     //String OBS1 = new String(message.data);
    OBS="";
    //ArrayList<String> obs_list = new ArrayList<String>();
    for(int j=0; j <  msg_list.size(); ++j)
       {
        OBS = OBS +  msg_list.get(j).toString() + ".";
       }
    System.out.println("SIZE="+ msg_list.size()+" and String OBS="+OBS);

    LSentence sd = parseSD();
    LSentence obs = parseOBS();
    LSentence indepModel = new LSentence();
    indepModel.addRules(sd);
    indepModel.addRules(obs);

    ArrayList result;
    
    int id = 0;
		ABTheoremProver thrmp = new ABTheoremProver();
    thrmp = indepModel.asABPropositionalSentence(thrmp);
    Iterator it = thrmp.getAssumptions().iterator();
    ArrayList<String> components = new ArrayList<String>();     
    while(it.hasNext()) {
            Assumption a = (Assumption)it.next();
            
            String ass = (String)a.identifier;
            boolean ass_ab = true;
            String compName = null;

            if (ass.matches(AB + "\\(" + "[a-zA-Z_0-9]+" + "\\)")) {

                compName = ass.substring(AB.length() + 1,
                                                ass.length() - 1);
                ass_ab = true;
            } else if (ass.matches(NAB + "\\(" + "[a-zA-Z_0-9]+" + "\\)")) {

                compName = ass.substring(NAB.length() + 1,
                                         ass.length() - 1);
                ass_ab = false;
             } //else if
             boolean present = false;
             for (int i = 0; i < components.size(); ++i) 
			       {    
                if(compName.equals(components.get(i).toString()))              
                    present = true;
              }
             if(!present)
               components.add(compName);
          }//while
         
        final Diagnosis dmsg = node.getMessageFactory().newMessage("diagnosis_msgs/Diagnosis");

         ArrayList<org.ros.message.diagnosis_msgs.DiagnosisResults>	diagArr = new                    				     ArrayList<org.ros.message.diagnosis_msgs.DiagnosisResults>();
         
        

        boolean consistent = checkConsistency(indepModel);
        String gd="",bd="";
        if (consistent) {
            org.ros.message.diagnosis_msgs.DiagnosisResults diag_result_c =  new org.ros.message.diagnosis_msgs.DiagnosisResults();
						ArrayList<String> good = new ArrayList<String>();  
            ArrayList<String> bad = new ArrayList<String>();
            result = new ArrayList();
            result.add("Consistent!");
            System.out.println("Consistent!");
            for(int j=0; j < components.size(); ++j)
                 {
                  gd= gd+"'"+components.get(j).toString()+"'";
                  good.add(components.get(j).toString());
                 }
            
                diag_result_c.good = good;
         				diag_result_c.bad  = bad;
         				dmsg.o_time =  now.getTimeInMillis();
         				diagArr.add(diag_result_c);
         				dmsg.diag = diagArr;
                
         				publisher.publish(dmsg);

        } else {  // inconsistent

          System.out.println("Not consistent!");
			    ArrayList diagnoses = new ArrayList();  // list of String
			    ArrayList conflictSets = new ArrayList();  // list of String
			    MinHittingSetsFM  hsFM=null;
			    try {
		         hsFM = new MinHittingSetsFM(false, thrmp, AB, NAB);
		         }catch (Exception e) {
	               System.out.println("Illigal assumptions");
			         }
		      int computationResult = hsFM.compute(10, 100);
			    boolean hasMoreDiags = (computationResult != MinHittingSets.CS_ALL_MIN_DIAGS_COMPUTED);
			    ArrayList minHittingSetsAsAss = hsFM.getMinHS();
			    ArrayList conflictsAsAss = hsFM.getConflictsAsAss();
        
        for (int i = 0; i < minHittingSetsAsAss.size(); ++i) 
			       { 
							 org.ros.message.diagnosis_msgs.DiagnosisResults diag_result_ic =  new org.ros.message.diagnosis_msgs.DiagnosisResults();           
               ArrayList<String> good1 = new ArrayList<String>();  
        			 ArrayList<String> bad1 = new ArrayList<String>();
							 gd="";bd="";
							 String res = minHittingSetsAsAss.get(i).toString();  
               for(int j=0; j < components.size(); ++j)
                 {
                  String comp = components.get(j).toString();
									boolean inComponents = res.indexOf(comp) > 0;
									if(!inComponents)
                      good1.add(comp);
                  else
                   bad1.add(comp);
                  
							} // for j
								
                             
          System.out.println(minHittingSetsAsAss.get(i));
					diag_result_ic.good = good1;
          diag_result_ic.bad  = bad1;
         	diagArr.add(diag_result_ic);	       
       }// for i
         
         dmsg.o_time =  now.getTimeInMillis();
         //diagArr.add(diag_result);
         dmsg.diag = diagArr;
         publisher.publish(dmsg);
        final DiagnosticArray d_arr_msg = node.getMessageFactory().newMessage("diagnostic_msgs/DiagnosticArray");
			  d_arr_msg.header.frame_id = "Engine";
        d_pub.publish(d_arr_msg);
       } // else inconsistent

     }catch (Exception e) {
       System.out.println("File Read Error!"+e);
       }
       
}// function

   
public void run() {
  try{
       while(true) {
         Thread.currentThread().sleep(50);
         processObs = true;
         Thread.currentThread().sleep(50);
         find_diag();
				 processObs = false;
         
        } // while true
   }
   catch(Exception e) {
         System.out.println(e);
   } 
 } // run

   protected ArrayList composeRepairCandidateResult(RepairCandidates rcs) {
        ArrayList result = new ArrayList(rcs.size());
        
        Iterator itCands = rcs.iterator();
        while (itCands.hasNext()) {
            RepairCandidate rc = (RepairCandidate)itCands.next();
            result.add(rc.toString());
        }

        return result;
    }

 protected LSentence parseSD() throws ParseError,
        IllegalUserInput{

        LogicParser parser = new LogicParser();
        LSentence allRules = new LSentence();

        generatePropNegationAxioms(PROP, parser, allRules);
        parseLogSentences(SD, parser, allRules);
  
        return allRules;
    }

 protected LSentence parseOBS() throws ParseError,
        IllegalUserInput{
        LogicParser parser = new LogicParser();
        LSentence allRules = new LSentence();

        parseLogSentences(OBS, parser, allRules);

        
        return allRules;
    }

protected void generatePropNegationAxioms(String text, LogicParser parser, 
                                              LSentence allRules) throws ParseError,
        IllegalUserInput
        {

        StringTokenizer tokenizer = new StringTokenizer(text, "\n");

        String negationPrefix = neg_prefix; //"n_";

        while(tokenizer.hasMoreTokens()) {
            String prop = tokenizer.nextToken().trim();
            
            if (prop.length() > 0) { 
                String line = prop + ", " + negationPrefix + prop + " -> false.";
                
                if (parser.parse(line)) {
                    allRules.addRules((LSentence)parser.result());
                } else throw new ParseError(prop);
            }
        }
    }


  protected void parseLogSentences(String text, LogicParser parser, LSentence allRules) throws ParseError,
        IllegalUserInput
        {

        //String text = textArea.getText();
        StringTokenizer tokenizer = new StringTokenizer(text, "\n");
        
        while(tokenizer.hasMoreTokens()) {
            String line = tokenizer.nextToken().trim();
            
            if (line.length() > 0) {                
                if (parser.parse(line)) {   
                    allRules.addRules((LSentence)parser.result());
                } else throw new ParseError(line);
            }
            
        } 
    }

  protected boolean checkConsistency(LSentence allRules) throws ParseError,
        IllegalUserInput {
        
        // prepare theorem prover, then check consistency

        ABTheoremProver theoremProver = new ABTheoremProver();
        theoremProver = allRules.asABPropositionalSentence(theoremProver);
        if (theoremProver == null) {
            throw new ParseError("[unknown]");
        }

        boolean consistent;
        boolean useFaultModelsCB = true;

        if (useFaultModelsCB==true) {
            String assAB = AB; 
            String assNAB = NAB; 

            if (assAB == null) {
                throw new IllegalUserInput("Invalid AB assumption defined!");
            } else if (assNAB == null) {
                throw new IllegalUserInput("Invalid NAB assumption defined!");
            }

            ArrayList posAssPrefixes = new ArrayList();
            posAssPrefixes.add(assNAB);
            consistent = theoremProver.checkConsistency(posAssPrefixes);
        } else {
            consistent = theoremProver.checkConsistency();
        }

        // print conflict set to console

        ArrayList conflict = theoremProver.contradiction().collectAssumptions();
        printConflictSet(conflict);

        return consistent;
    }

  protected void printConflictSet(ArrayList conflict) {        

        String cs = "";
        Iterator itC = conflict.iterator();
        int i=0;
        while (itC.hasNext()) {
            Assumption a = (Assumption)itC.next();
            if (cs.length() == 0) cs += "-";
            else cs += " \\/ -";
            cs += (String)a.identifier;
            i = i + 1;
        }
      if(i!=0)
        System.out.println("\n\nConflict set returned by theorem prover: " + cs + "\n\n");
    }

  @Override
  public void onShutdown(Node node) {
    node.shutdown();
    node = null;
  }

}
