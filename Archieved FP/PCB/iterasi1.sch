<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.5.1">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="10" visible="no" active="no"/>
<layer number="2" name="Route2" color="1" fill="3" visible="no" active="no"/>
<layer number="3" name="Route3" color="4" fill="3" visible="no" active="no"/>
<layer number="4" name="Route4" color="1" fill="4" visible="no" active="no"/>
<layer number="5" name="Route5" color="4" fill="4" visible="no" active="no"/>
<layer number="6" name="Route6" color="1" fill="8" visible="no" active="no"/>
<layer number="7" name="Route7" color="4" fill="8" visible="no" active="no"/>
<layer number="8" name="Route8" color="1" fill="2" visible="no" active="no"/>
<layer number="9" name="Route9" color="4" fill="2" visible="no" active="no"/>
<layer number="10" name="Route10" color="1" fill="7" visible="no" active="no"/>
<layer number="11" name="Route11" color="4" fill="7" visible="no" active="no"/>
<layer number="12" name="Route12" color="1" fill="5" visible="no" active="no"/>
<layer number="13" name="Route13" color="4" fill="5" visible="no" active="no"/>
<layer number="14" name="Route14" color="1" fill="6" visible="no" active="no"/>
<layer number="15" name="Route15" color="4" fill="6" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="26" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="con-lsta" urn="urn:adsk.eagle:library:161">
<description>&lt;b&gt;Female Headers etc.&lt;/b&gt;&lt;p&gt;
Naming:&lt;p&gt;
FE = female&lt;p&gt;
# contacts - # rows&lt;p&gt;
W = angled&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="FE06" urn="urn:adsk.eagle:footprint:8135/1" library_version="2">
<description>&lt;b&gt;FEMALE HEADER&lt;/b&gt;</description>
<wire x1="-7.62" y1="1.27" x2="-7.62" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="-1.27" x2="-5.334" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-5.334" y1="-1.27" x2="-5.08" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-1.016" x2="-4.826" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="-1.27" x2="-2.794" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.794" y1="-1.27" x2="-2.54" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="-1.016" x2="-2.286" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="-1.27" x2="-0.254" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-0.254" y1="-1.27" x2="0" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="0" y1="-1.016" x2="0.254" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0.254" y1="-1.27" x2="2.286" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.286" y1="-1.27" x2="2.54" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="2.54" y1="-1.016" x2="2.794" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="2.794" y1="-1.27" x2="4.826" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="5.334" y1="-1.27" x2="7.62" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="7.62" y1="-1.27" x2="7.62" y2="1.27" width="0.1524" layer="21"/>
<wire x1="7.62" y1="1.27" x2="5.334" y2="1.27" width="0.1524" layer="21"/>
<wire x1="4.826" y1="1.27" x2="2.794" y2="1.27" width="0.1524" layer="21"/>
<wire x1="2.794" y1="1.27" x2="2.54" y2="1.016" width="0.1524" layer="21"/>
<wire x1="2.54" y1="1.016" x2="2.286" y2="1.27" width="0.1524" layer="21"/>
<wire x1="2.286" y1="1.27" x2="0.254" y2="1.27" width="0.1524" layer="21"/>
<wire x1="0.254" y1="1.27" x2="0" y2="1.016" width="0.1524" layer="21"/>
<wire x1="0" y1="1.016" x2="-0.254" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-0.254" y1="1.27" x2="-2.286" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-2.286" y1="1.27" x2="-2.54" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-2.54" y1="1.016" x2="-2.794" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-2.794" y1="1.27" x2="-4.826" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-4.826" y1="1.27" x2="-5.08" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.016" x2="-5.334" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-5.334" y1="1.27" x2="-7.62" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="0.762" x2="-6.604" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-6.604" y1="0.508" x2="-6.858" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-6.858" y1="0.508" x2="-6.858" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-6.858" y1="-0.508" x2="-6.604" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-6.604" y1="-0.508" x2="-6.604" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-6.604" y1="-0.762" x2="-6.096" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-6.096" y1="-0.762" x2="-6.096" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-6.096" y1="-0.508" x2="-5.842" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-5.842" y1="-0.508" x2="-5.842" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-5.842" y1="0.508" x2="-6.096" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-6.096" y1="0.508" x2="-6.096" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-6.096" y1="0.762" x2="-6.604" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-4.064" y1="0.762" x2="-4.064" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-4.064" y1="0.508" x2="-4.318" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-4.318" y1="0.508" x2="-4.318" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-4.318" y1="-0.508" x2="-4.064" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-4.064" y1="-0.508" x2="-4.064" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-4.064" y1="-0.762" x2="-3.556" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-3.556" y1="-0.762" x2="-3.556" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-3.556" y1="-0.508" x2="-3.302" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-3.302" y1="-0.508" x2="-3.302" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-3.302" y1="0.508" x2="-3.556" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-3.556" y1="0.508" x2="-3.556" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-3.556" y1="0.762" x2="-4.064" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-1.524" y1="0.762" x2="-1.524" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-1.524" y1="0.508" x2="-1.778" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="0.508" x2="-1.778" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-1.778" y1="-0.508" x2="-1.524" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-1.524" y1="-0.508" x2="-1.524" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-1.524" y1="-0.762" x2="-1.016" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="-0.762" x2="-1.016" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="-0.508" x2="-0.762" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-0.762" y1="-0.508" x2="-0.762" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-0.762" y1="0.508" x2="-1.016" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="0.508" x2="-1.016" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-1.016" y1="0.762" x2="-1.524" y2="0.762" width="0.1524" layer="51"/>
<wire x1="1.016" y1="0.762" x2="1.016" y2="0.508" width="0.1524" layer="51"/>
<wire x1="1.016" y1="0.508" x2="0.762" y2="0.508" width="0.1524" layer="51"/>
<wire x1="0.762" y1="0.508" x2="0.762" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="0.762" y1="-0.508" x2="1.016" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="1.016" y1="-0.508" x2="1.016" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="1.016" y1="-0.762" x2="1.524" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="1.524" y1="-0.762" x2="1.524" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="1.524" y1="-0.508" x2="1.778" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="1.778" y1="-0.508" x2="1.778" y2="0.508" width="0.1524" layer="51"/>
<wire x1="1.778" y1="0.508" x2="1.524" y2="0.508" width="0.1524" layer="51"/>
<wire x1="1.524" y1="0.508" x2="1.524" y2="0.762" width="0.1524" layer="51"/>
<wire x1="1.524" y1="0.762" x2="1.016" y2="0.762" width="0.1524" layer="51"/>
<wire x1="3.556" y1="0.762" x2="3.556" y2="0.508" width="0.1524" layer="51"/>
<wire x1="3.556" y1="0.508" x2="3.302" y2="0.508" width="0.1524" layer="51"/>
<wire x1="3.302" y1="0.508" x2="3.302" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="3.302" y1="-0.508" x2="3.556" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="3.556" y1="-0.508" x2="3.556" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="3.556" y1="-0.762" x2="4.064" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="4.064" y1="-0.762" x2="4.064" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="4.064" y1="-0.508" x2="4.318" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="4.318" y1="-0.508" x2="4.318" y2="0.508" width="0.1524" layer="51"/>
<wire x1="4.318" y1="0.508" x2="4.064" y2="0.508" width="0.1524" layer="51"/>
<wire x1="4.064" y1="0.508" x2="4.064" y2="0.762" width="0.1524" layer="51"/>
<wire x1="4.064" y1="0.762" x2="3.556" y2="0.762" width="0.1524" layer="51"/>
<wire x1="5.334" y1="1.27" x2="5.08" y2="1.016" width="0.1524" layer="21"/>
<wire x1="5.08" y1="1.016" x2="4.826" y2="1.27" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-1.016" x2="5.334" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="4.826" y1="-1.27" x2="5.08" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="6.096" y1="0.762" x2="6.096" y2="0.508" width="0.1524" layer="51"/>
<wire x1="6.096" y1="0.508" x2="5.842" y2="0.508" width="0.1524" layer="51"/>
<wire x1="5.842" y1="0.508" x2="5.842" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="5.842" y1="-0.508" x2="6.096" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="6.096" y1="-0.508" x2="6.096" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="6.096" y1="-0.762" x2="6.604" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="6.604" y1="-0.762" x2="6.604" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="6.604" y1="-0.508" x2="6.858" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="6.858" y1="-0.508" x2="6.858" y2="0.508" width="0.1524" layer="51"/>
<wire x1="6.858" y1="0.508" x2="6.604" y2="0.508" width="0.1524" layer="51"/>
<wire x1="6.604" y1="0.508" x2="6.604" y2="0.762" width="0.1524" layer="51"/>
<wire x1="6.604" y1="0.762" x2="6.096" y2="0.762" width="0.1524" layer="51"/>
<pad name="1" x="-6.35" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="2" x="-3.81" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="3" x="-1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="4" x="1.27" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="5" x="3.81" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="6" x="6.35" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="-3.81" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-6.858" y="1.524" size="1.27" layer="21" ratio="10">1</text>
<text x="-7.62" y="-2.921" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="5.842" y="1.524" size="1.27" layer="21" ratio="10">6</text>
<rectangle x1="-6.477" y1="0.254" x2="-6.223" y2="0.762" layer="51"/>
<rectangle x1="-6.477" y1="-0.762" x2="-6.223" y2="-0.254" layer="51"/>
<rectangle x1="-3.937" y1="0.254" x2="-3.683" y2="0.762" layer="51"/>
<rectangle x1="-3.937" y1="-0.762" x2="-3.683" y2="-0.254" layer="51"/>
<rectangle x1="-1.397" y1="0.254" x2="-1.143" y2="0.762" layer="51"/>
<rectangle x1="-1.397" y1="-0.762" x2="-1.143" y2="-0.254" layer="51"/>
<rectangle x1="1.143" y1="0.254" x2="1.397" y2="0.762" layer="51"/>
<rectangle x1="1.143" y1="-0.762" x2="1.397" y2="-0.254" layer="51"/>
<rectangle x1="3.683" y1="0.254" x2="3.937" y2="0.762" layer="51"/>
<rectangle x1="3.683" y1="-0.762" x2="3.937" y2="-0.254" layer="51"/>
<rectangle x1="6.223" y1="0.254" x2="6.477" y2="0.762" layer="51"/>
<rectangle x1="6.223" y1="-0.762" x2="6.477" y2="-0.254" layer="51"/>
</package>
<package name="FE07" urn="urn:adsk.eagle:footprint:8137/1" library_version="2">
<description>&lt;b&gt;FEMALE HEADER&lt;/b&gt;</description>
<wire x1="-8.89" y1="1.27" x2="-8.89" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="-1.27" x2="-6.604" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="-1.27" x2="-6.35" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-6.35" y1="-1.016" x2="-6.096" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-6.096" y1="-1.27" x2="-4.064" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="-1.27" x2="-3.81" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-1.016" x2="-3.556" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-3.556" y1="-1.27" x2="-1.524" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="-1.27" x2="-1.27" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-1.016" x2="-1.016" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.016" y1="-1.27" x2="1.016" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.016" y1="-1.27" x2="1.27" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-1.016" x2="1.524" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="8.89" y1="-1.27" x2="8.89" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.524" y1="1.27" x2="1.27" y2="1.016" width="0.1524" layer="21"/>
<wire x1="1.27" y1="1.016" x2="1.016" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.016" y1="1.27" x2="-1.016" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.016" y1="1.27" x2="-1.27" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="1.016" x2="-1.524" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="1.27" x2="-3.556" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-3.556" y1="1.27" x2="-3.81" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="1.016" x2="-4.064" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-4.064" y1="1.27" x2="-6.096" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-6.096" y1="1.27" x2="-6.35" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-6.35" y1="1.016" x2="-6.604" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-6.604" y1="1.27" x2="-8.89" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-7.874" y1="0.762" x2="-7.874" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-7.874" y1="0.508" x2="-8.128" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-8.128" y1="0.508" x2="-8.128" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-8.128" y1="-0.508" x2="-7.874" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-7.874" y1="-0.508" x2="-7.874" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-7.874" y1="-0.762" x2="-7.366" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-7.366" y1="-0.762" x2="-7.366" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-7.366" y1="-0.508" x2="-7.112" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-7.112" y1="-0.508" x2="-7.112" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-7.112" y1="0.508" x2="-7.366" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-7.366" y1="0.508" x2="-7.366" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-7.366" y1="0.762" x2="-7.874" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-5.334" y1="0.762" x2="-5.334" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-5.334" y1="0.508" x2="-5.588" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-5.588" y1="0.508" x2="-5.588" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-5.588" y1="-0.508" x2="-5.334" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-5.334" y1="-0.508" x2="-5.334" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-5.334" y1="-0.762" x2="-4.826" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-4.826" y1="-0.762" x2="-4.826" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-4.826" y1="-0.508" x2="-4.572" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-4.572" y1="-0.508" x2="-4.572" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-4.572" y1="0.508" x2="-4.826" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-4.826" y1="0.508" x2="-4.826" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-4.826" y1="0.762" x2="-5.334" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-2.794" y1="0.762" x2="-2.794" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-2.794" y1="0.508" x2="-3.048" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-3.048" y1="0.508" x2="-3.048" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-3.048" y1="-0.508" x2="-2.794" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-2.794" y1="-0.508" x2="-2.794" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-2.794" y1="-0.762" x2="-2.286" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-2.286" y1="-0.762" x2="-2.286" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-2.286" y1="-0.508" x2="-2.032" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="-0.508" x2="-2.032" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-2.032" y1="0.508" x2="-2.286" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-2.286" y1="0.508" x2="-2.286" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-2.286" y1="0.762" x2="-2.794" y2="0.762" width="0.1524" layer="51"/>
<wire x1="-0.254" y1="0.762" x2="-0.254" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-0.254" y1="0.508" x2="-0.508" y2="0.508" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="0.508" x2="-0.508" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-0.508" y1="-0.508" x2="-0.254" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="-0.254" y1="-0.508" x2="-0.254" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="-0.254" y1="-0.762" x2="0.254" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="0.254" y1="-0.762" x2="0.254" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="0.254" y1="-0.508" x2="0.508" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="0.508" y1="-0.508" x2="0.508" y2="0.508" width="0.1524" layer="51"/>
<wire x1="0.508" y1="0.508" x2="0.254" y2="0.508" width="0.1524" layer="51"/>
<wire x1="0.254" y1="0.508" x2="0.254" y2="0.762" width="0.1524" layer="51"/>
<wire x1="0.254" y1="0.762" x2="-0.254" y2="0.762" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0.762" x2="2.286" y2="0.508" width="0.1524" layer="51"/>
<wire x1="2.286" y1="0.508" x2="2.032" y2="0.508" width="0.1524" layer="51"/>
<wire x1="2.032" y1="0.508" x2="2.032" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="2.032" y1="-0.508" x2="2.286" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="2.286" y1="-0.508" x2="2.286" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="2.286" y1="-0.762" x2="2.794" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="2.794" y1="-0.762" x2="2.794" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="2.794" y1="-0.508" x2="3.048" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="3.048" y1="-0.508" x2="3.048" y2="0.508" width="0.1524" layer="51"/>
<wire x1="3.048" y1="0.508" x2="2.794" y2="0.508" width="0.1524" layer="51"/>
<wire x1="2.794" y1="0.508" x2="2.794" y2="0.762" width="0.1524" layer="51"/>
<wire x1="2.794" y1="0.762" x2="2.286" y2="0.762" width="0.1524" layer="51"/>
<wire x1="1.524" y1="-1.27" x2="3.556" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="3.556" y1="-1.27" x2="3.81" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-1.016" x2="4.064" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="4.064" y1="-1.27" x2="6.096" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="6.096" y1="-1.27" x2="6.35" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="6.35" y1="-1.016" x2="6.604" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="6.604" y1="1.27" x2="6.35" y2="1.016" width="0.1524" layer="21"/>
<wire x1="6.35" y1="1.016" x2="6.096" y2="1.27" width="0.1524" layer="21"/>
<wire x1="6.096" y1="1.27" x2="4.064" y2="1.27" width="0.1524" layer="21"/>
<wire x1="4.064" y1="1.27" x2="3.81" y2="1.016" width="0.1524" layer="21"/>
<wire x1="3.81" y1="1.016" x2="3.556" y2="1.27" width="0.1524" layer="21"/>
<wire x1="3.556" y1="1.27" x2="1.524" y2="1.27" width="0.1524" layer="21"/>
<wire x1="4.826" y1="0.762" x2="4.826" y2="0.508" width="0.1524" layer="51"/>
<wire x1="4.826" y1="0.508" x2="4.572" y2="0.508" width="0.1524" layer="51"/>
<wire x1="4.572" y1="0.508" x2="4.572" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="4.572" y1="-0.508" x2="4.826" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="4.826" y1="-0.508" x2="4.826" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="4.826" y1="-0.762" x2="5.334" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="5.334" y1="-0.762" x2="5.334" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="5.334" y1="-0.508" x2="5.588" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="5.588" y1="-0.508" x2="5.588" y2="0.508" width="0.1524" layer="51"/>
<wire x1="5.588" y1="0.508" x2="5.334" y2="0.508" width="0.1524" layer="51"/>
<wire x1="5.334" y1="0.508" x2="5.334" y2="0.762" width="0.1524" layer="51"/>
<wire x1="5.334" y1="0.762" x2="4.826" y2="0.762" width="0.1524" layer="51"/>
<wire x1="7.366" y1="0.762" x2="7.366" y2="0.508" width="0.1524" layer="51"/>
<wire x1="7.366" y1="0.508" x2="7.112" y2="0.508" width="0.1524" layer="51"/>
<wire x1="7.112" y1="0.508" x2="7.112" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="7.112" y1="-0.508" x2="7.366" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="7.366" y1="-0.508" x2="7.366" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="7.366" y1="-0.762" x2="7.874" y2="-0.762" width="0.1524" layer="51"/>
<wire x1="7.874" y1="-0.762" x2="7.874" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="7.874" y1="-0.508" x2="8.128" y2="-0.508" width="0.1524" layer="51"/>
<wire x1="8.128" y1="-0.508" x2="8.128" y2="0.508" width="0.1524" layer="51"/>
<wire x1="8.128" y1="0.508" x2="7.874" y2="0.508" width="0.1524" layer="51"/>
<wire x1="7.874" y1="0.508" x2="7.874" y2="0.762" width="0.1524" layer="51"/>
<wire x1="7.874" y1="0.762" x2="7.366" y2="0.762" width="0.1524" layer="51"/>
<wire x1="8.89" y1="1.27" x2="6.604" y2="1.27" width="0.1524" layer="21"/>
<wire x1="6.604" y1="-1.27" x2="8.89" y2="-1.27" width="0.1524" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="2" x="-5.08" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="3" x="-2.54" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="4" x="0" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="5" x="2.54" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="6" x="5.08" y="0" drill="0.8128" shape="long" rot="R90"/>
<pad name="7" x="7.62" y="0" drill="0.8128" shape="long" rot="R90"/>
<text x="-5.08" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-8.128" y="1.524" size="1.27" layer="21" ratio="10">1</text>
<text x="-7.62" y="-2.921" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="7.112" y="1.524" size="1.27" layer="21" ratio="10">7</text>
<rectangle x1="-7.747" y1="0.254" x2="-7.493" y2="0.762" layer="51"/>
<rectangle x1="-7.747" y1="-0.762" x2="-7.493" y2="-0.254" layer="51"/>
<rectangle x1="-5.207" y1="0.254" x2="-4.953" y2="0.762" layer="51"/>
<rectangle x1="-5.207" y1="-0.762" x2="-4.953" y2="-0.254" layer="51"/>
<rectangle x1="-2.667" y1="0.254" x2="-2.413" y2="0.762" layer="51"/>
<rectangle x1="-2.667" y1="-0.762" x2="-2.413" y2="-0.254" layer="51"/>
<rectangle x1="-0.127" y1="0.254" x2="0.127" y2="0.762" layer="51"/>
<rectangle x1="-0.127" y1="-0.762" x2="0.127" y2="-0.254" layer="51"/>
<rectangle x1="2.413" y1="0.254" x2="2.667" y2="0.762" layer="51"/>
<rectangle x1="2.413" y1="-0.762" x2="2.667" y2="-0.254" layer="51"/>
<rectangle x1="4.953" y1="0.254" x2="5.207" y2="0.762" layer="51"/>
<rectangle x1="4.953" y1="-0.762" x2="5.207" y2="-0.254" layer="51"/>
<rectangle x1="7.493" y1="0.254" x2="7.747" y2="0.762" layer="51"/>
<rectangle x1="7.493" y1="-0.762" x2="7.747" y2="-0.254" layer="51"/>
</package>
</packages>
<packages3d>
<package3d name="FE06" urn="urn:adsk.eagle:package:8191/1" type="box" library_version="2">
<description>FEMALE HEADER</description>
<packageinstances>
<packageinstance name="FE06"/>
</packageinstances>
</package3d>
<package3d name="FE07" urn="urn:adsk.eagle:package:8223/1" type="box" library_version="2">
<description>FEMALE HEADER</description>
<packageinstances>
<packageinstance name="FE07"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="FE06-1" urn="urn:adsk.eagle:symbol:8134/1" library_version="2">
<wire x1="3.81" y1="-7.62" x2="-1.27" y2="-7.62" width="0.4064" layer="94"/>
<wire x1="1.905" y1="0.635" x2="1.905" y2="-0.635" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="-1.905" x2="1.905" y2="-3.175" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="-4.445" x2="1.905" y2="-5.715" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="-1.27" y1="10.16" x2="-1.27" y2="-7.62" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-7.62" x2="3.81" y2="10.16" width="0.4064" layer="94"/>
<wire x1="-1.27" y1="10.16" x2="3.81" y2="10.16" width="0.4064" layer="94"/>
<wire x1="1.905" y1="5.715" x2="1.905" y2="4.445" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="3.175" x2="1.905" y2="1.905" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="8.255" x2="1.905" y2="6.985" width="0.254" layer="94" curve="-180" cap="flat"/>
<text x="-1.27" y="-10.16" size="1.778" layer="96">&gt;VALUE</text>
<text x="-1.27" y="10.922" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="7.62" y="-5.08" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="3" x="7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="4" x="7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="5" x="7.62" y="5.08" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="6" x="7.62" y="7.62" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
<symbol name="FE07-1" urn="urn:adsk.eagle:symbol:8136/1" library_version="2">
<wire x1="3.81" y1="-10.16" x2="-1.27" y2="-10.16" width="0.4064" layer="94"/>
<wire x1="1.905" y1="-1.905" x2="1.905" y2="-3.175" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="-4.445" x2="1.905" y2="-5.715" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="-6.985" x2="1.905" y2="-8.255" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="-1.27" y1="10.16" x2="-1.27" y2="-10.16" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-10.16" x2="3.81" y2="10.16" width="0.4064" layer="94"/>
<wire x1="-1.27" y1="10.16" x2="3.81" y2="10.16" width="0.4064" layer="94"/>
<wire x1="1.905" y1="3.175" x2="1.905" y2="1.905" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="0.635" x2="1.905" y2="-0.635" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="5.715" x2="1.905" y2="4.445" width="0.254" layer="94" curve="-180" cap="flat"/>
<wire x1="1.905" y1="8.255" x2="1.905" y2="6.985" width="0.254" layer="94" curve="-180" cap="flat"/>
<text x="-1.27" y="-12.7" size="1.778" layer="96">&gt;VALUE</text>
<text x="-1.27" y="10.922" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="7.62" y="-7.62" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="7.62" y="-5.08" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="3" x="7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="4" x="7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="5" x="7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="6" x="7.62" y="5.08" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="7" x="7.62" y="7.62" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="FE06-1" urn="urn:adsk.eagle:component:8235/2" prefix="SV" uservalue="yes" library_version="2">
<description>&lt;b&gt;FEMALE HEADER&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="FE06-1" x="0" y="0"/>
</gates>
<devices>
<device name="" package="FE06">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8191/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
<attribute name="POPULARITY" value="11" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="FE07-1" urn="urn:adsk.eagle:component:8261/2" prefix="SV" uservalue="yes" library_version="2">
<description>&lt;b&gt;FEMALE HEADER&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="FE07-1" x="0" y="0"/>
</gates>
<devices>
<device name="" package="FE07">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
<connect gate="G$1" pin="7" pad="7"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8223/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
<attribute name="POPULARITY" value="3" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="con-lstb" urn="urn:adsk.eagle:library:162">
<description>&lt;b&gt;Pin Headers&lt;/b&gt;&lt;p&gt;
Naming:&lt;p&gt;
MA = male&lt;p&gt;
# contacts - # rows&lt;p&gt;
W = angled&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="MA03-1" urn="urn:adsk.eagle:footprint:8281/1" library_version="2">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<wire x1="-3.175" y1="1.27" x2="-1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="1.27" x2="-1.27" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="-0.635" x2="-1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.27" y1="0.635" x2="-0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="1.27" x2="0.635" y2="1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="1.27" x2="1.27" y2="0.635" width="0.1524" layer="21"/>
<wire x1="1.27" y1="-0.635" x2="0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-1.27" x2="-0.635" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-1.27" x2="-1.27" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="0.635" x2="-3.81" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-3.175" y1="1.27" x2="-3.81" y2="0.635" width="0.1524" layer="21"/>
<wire x1="-3.81" y1="-0.635" x2="-3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="-1.905" y1="-1.27" x2="-3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.27" y1="0.635" x2="1.905" y2="1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="1.27" x2="3.175" y2="1.27" width="0.1524" layer="21"/>
<wire x1="3.175" y1="1.27" x2="3.81" y2="0.635" width="0.1524" layer="21"/>
<wire x1="3.81" y1="-0.635" x2="3.175" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="3.175" y1="-1.27" x2="1.905" y2="-1.27" width="0.1524" layer="21"/>
<wire x1="1.905" y1="-1.27" x2="1.27" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="3.81" y1="0.635" x2="3.81" y2="-0.635" width="0.1524" layer="21"/>
<pad name="1" x="-2.54" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="2" x="0" y="0" drill="1.016" shape="long" rot="R90"/>
<pad name="3" x="2.54" y="0" drill="1.016" shape="long" rot="R90"/>
<text x="-3.81" y="1.651" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-5.08" y="-0.635" size="1.27" layer="21" ratio="10">1</text>
<text x="-3.81" y="-2.921" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-0.254" y1="-0.254" x2="0.254" y2="0.254" layer="51"/>
<rectangle x1="-2.794" y1="-0.254" x2="-2.286" y2="0.254" layer="51"/>
<rectangle x1="2.286" y1="-0.254" x2="2.794" y2="0.254" layer="51"/>
</package>
</packages>
<packages3d>
<package3d name="MA03-1" urn="urn:adsk.eagle:package:8339/1" type="box" library_version="2">
<description>PIN HEADER</description>
<packageinstances>
<packageinstance name="MA03-1"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="MA03-1" urn="urn:adsk.eagle:symbol:8280/1" library_version="2">
<wire x1="3.81" y1="-5.08" x2="-1.27" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="1.27" y1="2.54" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="1.27" y1="0" x2="2.54" y2="0" width="0.6096" layer="94"/>
<wire x1="1.27" y1="-2.54" x2="2.54" y2="-2.54" width="0.6096" layer="94"/>
<wire x1="-1.27" y1="5.08" x2="-1.27" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="3.81" y1="-5.08" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<wire x1="-1.27" y1="5.08" x2="3.81" y2="5.08" width="0.4064" layer="94"/>
<text x="-1.27" y="-7.62" size="1.778" layer="96">&gt;VALUE</text>
<text x="-1.27" y="5.842" size="1.778" layer="95">&gt;NAME</text>
<pin name="1" x="7.62" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="2" x="7.62" y="0" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
<pin name="3" x="7.62" y="2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="MA03-1" urn="urn:adsk.eagle:component:8376/2" prefix="SV" uservalue="yes" library_version="2">
<description>&lt;b&gt;PIN HEADER&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="MA03-1" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MA03-1">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8339/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
<attribute name="POPULARITY" value="48" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="con-wago-500" urn="urn:adsk.eagle:library:195">
<description>&lt;b&gt;Wago Screw Clamps&lt;/b&gt;&lt;p&gt;
Grid 5.00 mm&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="W237-103" urn="urn:adsk.eagle:footprint:10678/1" library_version="2">
<description>&lt;b&gt;WAGO SCREW CLAMP&lt;/b&gt;</description>
<wire x1="4.093" y1="-2.255" x2="5.897" y2="-0.299" width="0.254" layer="51"/>
<wire x1="-0.911" y1="-2.331" x2="0.994" y2="-0.299" width="0.254" layer="51"/>
<wire x1="-5.991" y1="-2.306" x2="-3.984" y2="-0.299" width="0.254" layer="51"/>
<wire x1="-7.489" y1="-5.481" x2="7.497" y2="-5.481" width="0.1524" layer="21"/>
<wire x1="7.497" y1="3.714" x2="7.497" y2="3.511" width="0.1524" layer="21"/>
<wire x1="7.497" y1="3.714" x2="-7.489" y2="3.714" width="0.1524" layer="21"/>
<wire x1="-7.489" y1="-5.481" x2="-7.489" y2="-3.093" width="0.1524" layer="21"/>
<wire x1="-7.489" y1="-3.093" x2="-5.889" y2="-3.093" width="0.1524" layer="21"/>
<wire x1="-5.889" y1="-3.093" x2="-4.111" y2="-3.093" width="0.1524" layer="51"/>
<wire x1="-4.111" y1="-3.093" x2="-0.885" y2="-3.093" width="0.1524" layer="21"/>
<wire x1="0.893" y1="-3.093" x2="4.119" y2="-3.093" width="0.1524" layer="21"/>
<wire x1="5.897" y1="-3.093" x2="7.497" y2="-3.093" width="0.1524" layer="21"/>
<wire x1="-7.489" y1="-3.093" x2="-7.489" y2="3.511" width="0.1524" layer="21"/>
<wire x1="7.497" y1="-3.093" x2="7.497" y2="-5.481" width="0.1524" layer="21"/>
<wire x1="7.497" y1="3.511" x2="-7.489" y2="3.511" width="0.1524" layer="21"/>
<wire x1="7.497" y1="3.511" x2="7.497" y2="-3.093" width="0.1524" layer="21"/>
<wire x1="-7.489" y1="3.511" x2="-7.489" y2="3.714" width="0.1524" layer="21"/>
<wire x1="-0.885" y1="-3.093" x2="0.893" y2="-3.093" width="0.1524" layer="51"/>
<wire x1="4.119" y1="-3.093" x2="5.897" y2="-3.093" width="0.1524" layer="51"/>
<circle x="-5" y="-1.29" radius="1.4986" width="0.1524" layer="51"/>
<circle x="5.0076" y="-1.29" radius="1.4986" width="0.1524" layer="51"/>
<circle x="-5" y="2.1898" radius="0.508" width="0.1524" layer="21"/>
<circle x="5.0076" y="2.1898" radius="0.508" width="0.1524" layer="21"/>
<circle x="0.0038" y="-1.29" radius="1.4986" width="0.1524" layer="51"/>
<circle x="0.0038" y="2.1898" radius="0.508" width="0.1524" layer="21"/>
<pad name="1" x="-5" y="-1.29" drill="1.1938" shape="long" rot="R90"/>
<pad name="2" x="0" y="-1.29" drill="1.1938" shape="long" rot="R90"/>
<pad name="3" x="5" y="-1.29" drill="1.1938" shape="long" rot="R90"/>
<text x="-6.905" y="0.615" size="1.27" layer="51" ratio="10">1</text>
<text x="-1.8504" y="0.5642" size="1.27" layer="51" ratio="10">2</text>
<text x="3.1534" y="0.615" size="1.27" layer="51" ratio="10">3</text>
<text x="-5.3048" y="-4.9476" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-5.6858" y="-7.4622" size="1.27" layer="25" ratio="10">&gt;NAME</text>
</package>
<package name="W237-102" urn="urn:adsk.eagle:footprint:10676/1" library_version="2">
<description>&lt;b&gt;WAGO SCREW CLAMP&lt;/b&gt;</description>
<wire x1="-3.491" y1="-2.286" x2="-1.484" y2="-0.279" width="0.254" layer="51"/>
<wire x1="1.488" y1="-2.261" x2="3.469" y2="-0.254" width="0.254" layer="51"/>
<wire x1="-4.989" y1="-5.461" x2="4.993" y2="-5.461" width="0.1524" layer="21"/>
<wire x1="4.993" y1="3.734" x2="4.993" y2="3.531" width="0.1524" layer="21"/>
<wire x1="4.993" y1="3.734" x2="-4.989" y2="3.734" width="0.1524" layer="21"/>
<wire x1="-4.989" y1="-5.461" x2="-4.989" y2="-3.073" width="0.1524" layer="21"/>
<wire x1="-4.989" y1="-3.073" x2="-3.389" y2="-3.073" width="0.1524" layer="21"/>
<wire x1="-3.389" y1="-3.073" x2="-1.611" y2="-3.073" width="0.1524" layer="51"/>
<wire x1="-1.611" y1="-3.073" x2="1.615" y2="-3.073" width="0.1524" layer="21"/>
<wire x1="3.393" y1="-3.073" x2="4.993" y2="-3.073" width="0.1524" layer="21"/>
<wire x1="-4.989" y1="-3.073" x2="-4.989" y2="3.531" width="0.1524" layer="21"/>
<wire x1="4.993" y1="-3.073" x2="4.993" y2="-5.461" width="0.1524" layer="21"/>
<wire x1="-4.989" y1="3.531" x2="4.993" y2="3.531" width="0.1524" layer="21"/>
<wire x1="-4.989" y1="3.531" x2="-4.989" y2="3.734" width="0.1524" layer="21"/>
<wire x1="4.993" y1="3.531" x2="4.993" y2="-3.073" width="0.1524" layer="21"/>
<wire x1="1.615" y1="-3.073" x2="3.393" y2="-3.073" width="0.1524" layer="51"/>
<circle x="-2.5" y="-1.27" radius="1.4986" width="0.1524" layer="51"/>
<circle x="-2.5" y="2.2098" radius="0.508" width="0.1524" layer="21"/>
<circle x="2.5038" y="-1.27" radius="1.4986" width="0.1524" layer="51"/>
<circle x="2.5038" y="2.2098" radius="0.508" width="0.1524" layer="21"/>
<pad name="1" x="-2.5" y="-1.27" drill="1.1938" shape="long" rot="R90"/>
<pad name="2" x="2.5" y="-1.27" drill="1.1938" shape="long" rot="R90"/>
<text x="-5.04" y="-7.62" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-3.8462" y="-5.0038" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.532" y="0.635" size="1.27" layer="21" ratio="10">1</text>
<text x="0.421" y="0.635" size="1.27" layer="21" ratio="10">2</text>
</package>
<package name="W237-4" urn="urn:adsk.eagle:footprint:10684/1" library_version="2">
<description>&lt;b&gt;WAGO SCREW CLAMP&lt;/b&gt;</description>
<wire x1="-8.491" y1="-2.286" x2="-6.484" y2="-0.279" width="0.254" layer="51"/>
<wire x1="-3.512" y1="-2.261" x2="-1.531" y2="-0.254" width="0.254" layer="51"/>
<wire x1="1.517" y1="-2.286" x2="3.524" y2="-0.279" width="0.254" layer="51"/>
<wire x1="6.495" y1="-2.261" x2="8.477" y2="-0.254" width="0.254" layer="51"/>
<wire x1="-9.989" y1="-5.461" x2="10.001" y2="-5.461" width="0.1524" layer="21"/>
<wire x1="10.001" y1="3.734" x2="10.001" y2="3.531" width="0.1524" layer="21"/>
<wire x1="10.001" y1="3.734" x2="-9.989" y2="3.734" width="0.1524" layer="21"/>
<wire x1="-9.989" y1="-5.461" x2="-9.989" y2="-3.073" width="0.1524" layer="21"/>
<wire x1="-9.989" y1="-3.073" x2="-8.389" y2="-3.073" width="0.1524" layer="21"/>
<wire x1="-8.389" y1="-3.073" x2="-6.611" y2="-3.073" width="0.1524" layer="51"/>
<wire x1="-6.611" y1="-3.073" x2="-3.385" y2="-3.073" width="0.1524" layer="21"/>
<wire x1="-1.607" y1="-3.073" x2="1.619" y2="-3.073" width="0.1524" layer="21"/>
<wire x1="3.397" y1="-3.073" x2="6.622" y2="-3.073" width="0.1524" layer="21"/>
<wire x1="8.4" y1="-3.073" x2="10.001" y2="-3.073" width="0.1524" layer="21"/>
<wire x1="-9.989" y1="-3.073" x2="-9.989" y2="3.531" width="0.1524" layer="21"/>
<wire x1="10.001" y1="-3.073" x2="10.001" y2="-5.461" width="0.1524" layer="21"/>
<wire x1="-9.989" y1="3.531" x2="10.001" y2="3.531" width="0.1524" layer="21"/>
<wire x1="-9.989" y1="3.531" x2="-9.989" y2="3.734" width="0.1524" layer="21"/>
<wire x1="10.001" y1="3.531" x2="10.001" y2="-3.073" width="0.1524" layer="21"/>
<wire x1="-3.385" y1="-3.073" x2="-1.607" y2="-3.073" width="0.1524" layer="51"/>
<wire x1="1.619" y1="-3.073" x2="3.397" y2="-3.073" width="0.1524" layer="51"/>
<wire x1="6.622" y1="-3.073" x2="8.4" y2="-3.073" width="0.1524" layer="51"/>
<circle x="-7.5" y="-1.27" radius="1.4986" width="0.1524" layer="51"/>
<circle x="-7.5" y="2.2098" radius="0.508" width="0.1524" layer="21"/>
<circle x="-2.4962" y="-1.27" radius="1.4986" width="0.1524" layer="51"/>
<circle x="-2.4962" y="2.2098" radius="0.508" width="0.1524" layer="21"/>
<circle x="2.5076" y="-1.27" radius="1.4986" width="0.1524" layer="51"/>
<circle x="2.5076" y="2.2098" radius="0.508" width="0.1524" layer="21"/>
<circle x="7.5114" y="-1.27" radius="1.4986" width="0.1524" layer="51"/>
<circle x="7.5114" y="2.2098" radius="0.508" width="0.1524" layer="21"/>
<pad name="1" x="-7.5" y="-1.27" drill="1.1938" shape="long" rot="R90"/>
<pad name="2" x="-2.5" y="-1.27" drill="1.1938" shape="long" rot="R90"/>
<pad name="3" x="2.5" y="-1.27" drill="1.1938" shape="long" rot="R90"/>
<pad name="4" x="7.5" y="-1.27" drill="1.1938" shape="long" rot="R90"/>
<text x="-7.6524" y="-5.0292" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-8.7446" y="-7.4422" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-9.532" y="0.635" size="1.27" layer="51" ratio="10">1</text>
<text x="-4.579" y="0.635" size="1.27" layer="51" ratio="10">2</text>
<text x="0.4756" y="0.635" size="1.27" layer="51" ratio="10">3</text>
<text x="5.4286" y="0.635" size="1.27" layer="51" ratio="10">4</text>
</package>
</packages>
<packages3d>
<package3d name="W237-103" urn="urn:adsk.eagle:package:10691/1" type="box" library_version="2">
<description>WAGO SCREW CLAMP</description>
<packageinstances>
<packageinstance name="W237-103"/>
</packageinstances>
</package3d>
<package3d name="W237-102" urn="urn:adsk.eagle:package:10688/1" type="box" library_version="2">
<description>WAGO SCREW CLAMP</description>
<packageinstances>
<packageinstance name="W237-102"/>
</packageinstances>
</package3d>
<package3d name="W237-4" urn="urn:adsk.eagle:package:10695/1" type="box" library_version="2">
<description>WAGO SCREW CLAMP</description>
<packageinstances>
<packageinstance name="W237-4"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="KL" urn="urn:adsk.eagle:symbol:10675/1" library_version="2">
<circle x="1.27" y="0" radius="1.27" width="0.254" layer="94"/>
<text x="0" y="0.889" size="1.778" layer="95" rot="R180">&gt;NAME</text>
<pin name="KL" x="5.08" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
<symbol name="KL+V" urn="urn:adsk.eagle:symbol:10673/1" library_version="2">
<circle x="1.27" y="0" radius="1.27" width="0.254" layer="94"/>
<text x="-2.54" y="-3.683" size="1.778" layer="96">&gt;VALUE</text>
<text x="0" y="0.889" size="1.778" layer="95" rot="R180">&gt;NAME</text>
<pin name="KL" x="5.08" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="W237-103" urn="urn:adsk.eagle:component:10707/2" prefix="X" uservalue="yes" library_version="2">
<description>&lt;b&gt;WAGO SCREW CLAMP&lt;/b&gt;</description>
<gates>
<gate name="-1" symbol="KL" x="0" y="5.08" addlevel="always"/>
<gate name="-2" symbol="KL" x="0" y="0" addlevel="always"/>
<gate name="-3" symbol="KL+V" x="0" y="-5.08" addlevel="always"/>
</gates>
<devices>
<device name="" package="W237-103">
<connects>
<connect gate="-1" pin="KL" pad="1"/>
<connect gate="-2" pin="KL" pad="2"/>
<connect gate="-3" pin="KL" pad="3"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:10691/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="237-103" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="18M7116" constant="no"/>
<attribute name="POPULARITY" value="5" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="W237-102" urn="urn:adsk.eagle:component:10702/2" prefix="X" uservalue="yes" library_version="2">
<description>&lt;b&gt;WAGO SCREW CLAMP&lt;/b&gt;</description>
<gates>
<gate name="-1" symbol="KL" x="0" y="5.08" addlevel="always"/>
<gate name="-2" symbol="KL+V" x="0" y="0" addlevel="always"/>
</gates>
<devices>
<device name="" package="W237-102">
<connects>
<connect gate="-1" pin="KL" pad="1"/>
<connect gate="-2" pin="KL" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:10688/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="237-102" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="70K9898" constant="no"/>
<attribute name="POPULARITY" value="32" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="W237-4" urn="urn:adsk.eagle:component:10705/2" prefix="X" uservalue="yes" library_version="2">
<description>&lt;b&gt;WAGO SCREW CLAMP&lt;/b&gt;</description>
<gates>
<gate name="-1" symbol="KL" x="0" y="10.16" addlevel="always"/>
<gate name="-2" symbol="KL" x="0" y="5.08" addlevel="always"/>
<gate name="-3" symbol="KL" x="0" y="0" addlevel="always"/>
<gate name="-4" symbol="KL+V" x="0" y="-5.08" addlevel="always"/>
</gates>
<devices>
<device name="" package="W237-4">
<connects>
<connect gate="-1" pin="KL" pad="1"/>
<connect gate="-2" pin="KL" pad="2"/>
<connect gate="-3" pin="KL" pad="3"/>
<connect gate="-4" pin="KL" pad="4"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:10695/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
<attribute name="POPULARITY" value="10" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="con-molex" urn="urn:adsk.eagle:library:165">
<description>&lt;b&gt;Molex Connectors&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="22-23-2101" urn="urn:adsk.eagle:footprint:8078267/1" library_version="4">
<description>&lt;b&gt;KK® 254 Solid Header, Vertical, with Friction Lock, 10 Circuits, Tin (Sn) Plating&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022232101_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<wire x1="-12.7" y1="3.175" x2="12.7" y2="3.175" width="0.254" layer="21"/>
<wire x1="12.7" y1="3.175" x2="12.7" y2="1.27" width="0.254" layer="21"/>
<wire x1="12.7" y1="1.27" x2="12.7" y2="-3.175" width="0.254" layer="21"/>
<wire x1="12.7" y1="-3.175" x2="-12.7" y2="-3.175" width="0.254" layer="21"/>
<wire x1="-12.7" y1="-3.175" x2="-12.7" y2="1.27" width="0.254" layer="21"/>
<wire x1="-12.7" y1="1.27" x2="-12.7" y2="3.175" width="0.254" layer="21"/>
<wire x1="-12.7" y1="1.27" x2="12.7" y2="1.27" width="0.254" layer="21"/>
<pad name="1" x="-11.43" y="0" drill="1" shape="long" rot="R90"/>
<pad name="2" x="-8.89" y="0" drill="1" shape="long" rot="R90"/>
<pad name="3" x="-6.35" y="0" drill="1" shape="long" rot="R90"/>
<pad name="4" x="-3.81" y="0" drill="1" shape="long" rot="R90"/>
<pad name="5" x="-1.27" y="0" drill="1" shape="long" rot="R90"/>
<pad name="6" x="1.27" y="0" drill="1" shape="long" rot="R90"/>
<pad name="7" x="3.81" y="0" drill="1" shape="long" rot="R90"/>
<pad name="8" x="6.35" y="0" drill="1" shape="long" rot="R90"/>
<pad name="9" x="8.89" y="0" drill="1" shape="long" rot="R90"/>
<pad name="10" x="11.43" y="0" drill="1" shape="long" rot="R90"/>
<text x="-12.7" y="3.81" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-12.7" y="-5.08" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="22-23-2021" urn="urn:adsk.eagle:footprint:8078259/1" library_version="4">
<description>&lt;b&gt;KK® 254 Solid Header, Vertical, with Friction Lock, 2 Circuits, Tin (Sn) Plating&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022232021_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<wire x1="-2.54" y1="3.175" x2="2.54" y2="3.175" width="0.254" layer="21"/>
<wire x1="2.54" y1="3.175" x2="2.54" y2="1.27" width="0.254" layer="21"/>
<wire x1="2.54" y1="1.27" x2="2.54" y2="-3.175" width="0.254" layer="21"/>
<wire x1="2.54" y1="-3.175" x2="-2.54" y2="-3.175" width="0.254" layer="21"/>
<wire x1="-2.54" y1="-3.175" x2="-2.54" y2="1.27" width="0.254" layer="21"/>
<wire x1="-2.54" y1="1.27" x2="-2.54" y2="3.175" width="0.254" layer="21"/>
<wire x1="-2.54" y1="1.27" x2="2.54" y2="1.27" width="0.254" layer="21"/>
<pad name="1" x="-1.27" y="0" drill="1" shape="long" rot="R90"/>
<pad name="2" x="1.27" y="0" drill="1" shape="long" rot="R90"/>
<text x="-2.54" y="3.81" size="1.016" layer="25" ratio="10">&gt;NAME</text>
<text x="-2.54" y="-5.08" size="1.016" layer="27" ratio="10">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="22-23-2101" urn="urn:adsk.eagle:package:8078641/1" type="box" library_version="4">
<description>&lt;b&gt;KK® 254 Solid Header, Vertical, with Friction Lock, 10 Circuits, Tin (Sn) Plating&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022232101_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<packageinstances>
<packageinstance name="22-23-2101"/>
</packageinstances>
</package3d>
<package3d name="22-23-2021" urn="urn:adsk.eagle:package:8078633/1" type="box" library_version="4">
<description>&lt;b&gt;KK® 254 Solid Header, Vertical, with Friction Lock, 2 Circuits, Tin (Sn) Plating&lt;/b&gt;&lt;p&gt;&lt;a href =http://www.molex.com/pdm_docs/sd/022232021_sd.pdf&gt;Datasheet &lt;/a&gt;</description>
<packageinstances>
<packageinstance name="22-23-2021"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="MV" urn="urn:adsk.eagle:symbol:8078125/1" library_version="4">
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.6096" layer="94"/>
<text x="2.54" y="-0.762" size="1.524" layer="95">&gt;NAME</text>
<text x="-0.762" y="1.397" size="1.778" layer="96">&gt;VALUE</text>
<pin name="S" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
</symbol>
<symbol name="M" urn="urn:adsk.eagle:symbol:8078124/1" library_version="4">
<wire x1="1.27" y1="0" x2="0" y2="0" width="0.6096" layer="94"/>
<text x="2.54" y="-0.762" size="1.524" layer="95">&gt;NAME</text>
<pin name="S" x="-2.54" y="0" visible="off" length="short" direction="pas"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="22-23-2101" urn="urn:adsk.eagle:component:8078966/2" prefix="X" library_version="4">
<description>.100" (2.54mm) Center Header - 10 Pin</description>
<gates>
<gate name="-1" symbol="MV" x="0" y="10.16" addlevel="always" swaplevel="1"/>
<gate name="-2" symbol="M" x="0" y="7.62" addlevel="always" swaplevel="1"/>
<gate name="-3" symbol="M" x="0" y="5.08" addlevel="always" swaplevel="1"/>
<gate name="-4" symbol="M" x="0" y="2.54" addlevel="always" swaplevel="1"/>
<gate name="-5" symbol="M" x="0" y="0" addlevel="always" swaplevel="1"/>
<gate name="-6" symbol="M" x="0" y="-2.54" addlevel="always" swaplevel="1"/>
<gate name="-7" symbol="M" x="0" y="-5.08" addlevel="always" swaplevel="1"/>
<gate name="-8" symbol="M" x="0" y="-7.62" addlevel="always" swaplevel="1"/>
<gate name="-9" symbol="M" x="0" y="-10.16" addlevel="always" swaplevel="1"/>
<gate name="-10" symbol="M" x="0" y="-12.7" addlevel="always" swaplevel="1"/>
</gates>
<devices>
<device name="" package="22-23-2101">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-10" pin="S" pad="10"/>
<connect gate="-2" pin="S" pad="2"/>
<connect gate="-3" pin="S" pad="3"/>
<connect gate="-4" pin="S" pad="4"/>
<connect gate="-5" pin="S" pad="5"/>
<connect gate="-6" pin="S" pad="6"/>
<connect gate="-7" pin="S" pad="7"/>
<connect gate="-8" pin="S" pad="8"/>
<connect gate="-9" pin="S" pad="9"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8078641/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="MOLEX" constant="no"/>
<attribute name="MPN" value="22-23-2101" constant="no"/>
<attribute name="OC_FARNELL" value="1756828" constant="no"/>
<attribute name="OC_NEWARK" value="94C0765" constant="no"/>
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="22-23-2021" urn="urn:adsk.eagle:component:8078938/2" prefix="X" library_version="4">
<description>.100" (2.54mm) Center Header - 2 Pin</description>
<gates>
<gate name="-1" symbol="MV" x="0" y="0" addlevel="always" swaplevel="1"/>
<gate name="-2" symbol="M" x="0" y="-2.54" addlevel="always" swaplevel="1"/>
</gates>
<devices>
<device name="" package="22-23-2021">
<connects>
<connect gate="-1" pin="S" pad="1"/>
<connect gate="-2" pin="S" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8078633/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="MOLEX" constant="no"/>
<attribute name="MPN" value="22-23-2021" constant="no"/>
<attribute name="OC_FARNELL" value="1462926" constant="no"/>
<attribute name="OC_NEWARK" value="25C3832" constant="no"/>
<attribute name="POPULARITY" value="40" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<groups>
<schematic_group name="RELAY" grouprefs="GAS"/>
<schematic_group name="SWITCH" grouprefs="GAS"/>
<schematic_group name="TPS-MB" grouprefs="GAS"/>
<schematic_group name="GAS"/>
<schematic_group name="STIR"/>
<schematic_group name="REM"/>
</groups>
<parts>
<part name="DAC" library="con-lsta" library_urn="urn:adsk.eagle:library:161" deviceset="FE06-1" device="" package3d_urn="urn:adsk.eagle:package:8191/1"/>
<part name="RELAY-1" library="con-lstb" library_urn="urn:adsk.eagle:library:162" deviceset="MA03-1" device="" package3d_urn="urn:adsk.eagle:package:8339/1"/>
<part name="RELAY-2" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-103" device="" package3d_urn="urn:adsk.eagle:package:10691/1"/>
<part name="SW-IN" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="SW-OUT" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="TPS" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-103" device="" package3d_urn="urn:adsk.eagle:package:10691/1"/>
<part name="MB" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-103" device="" package3d_urn="urn:adsk.eagle:package:10691/1"/>
<part name="DRV-STEPPER" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-4" device="" package3d_urn="urn:adsk.eagle:package:10695/1"/>
<part name="REM-DRV" library="con-lsta" library_urn="urn:adsk.eagle:library:161" deviceset="FE07-1" device="" package3d_urn="urn:adsk.eagle:package:8223/1"/>
<part name="24V" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="REM-MTR" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="REM-KTC" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-103" device="" package3d_urn="urn:adsk.eagle:package:10691/1"/>
<part name="ENC" library="con-molex" library_urn="urn:adsk.eagle:library:165" deviceset="22-23-2101" device="" package3d_urn="urn:adsk.eagle:package:8078641/1"/>
<part name="12V" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="ENC2" library="con-molex" library_urn="urn:adsk.eagle:library:165" deviceset="22-23-2021" device="" package3d_urn="urn:adsk.eagle:package:8078633/1"/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="DAC" gate="G$1" x="53.34" y="83.82" smashed="yes" grouprefs="GAS">
<attribute name="VALUE" x="52.07" y="73.66" size="1.778" layer="96"/>
<attribute name="NAME" x="52.07" y="94.742" size="1.778" layer="95"/>
</instance>
<instance part="RELAY-1" gate="G$1" x="15.24" y="109.22" smashed="yes" grouprefs="RELAY GAS">
<attribute name="VALUE" x="13.97" y="101.6" size="1.778" layer="96"/>
<attribute name="NAME" x="13.97" y="115.062" size="1.778" layer="95"/>
</instance>
<instance part="RELAY-2" gate="-1" x="50.8" y="114.3" smashed="yes" grouprefs="RELAY GAS">
<attribute name="NAME" x="50.8" y="115.189" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="RELAY-2" gate="-2" x="50.8" y="109.22" smashed="yes" grouprefs="RELAY GAS">
<attribute name="NAME" x="50.8" y="110.109" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="RELAY-2" gate="-3" x="50.8" y="104.14" smashed="yes" grouprefs="RELAY GAS">
<attribute name="VALUE" x="48.26" y="100.457" size="1.778" layer="96"/>
<attribute name="NAME" x="50.8" y="105.029" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="SW-IN" gate="-1" x="25.4" y="134.62" smashed="yes" grouprefs="SWITCH GAS">
<attribute name="NAME" x="25.4" y="135.509" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="SW-IN" gate="-2" x="25.4" y="129.54" smashed="yes" grouprefs="SWITCH GAS">
<attribute name="VALUE" x="22.86" y="125.857" size="1.778" layer="96"/>
<attribute name="NAME" x="25.4" y="130.429" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="SW-OUT" gate="-1" x="53.34" y="134.62" smashed="yes" grouprefs="SWITCH GAS">
<attribute name="NAME" x="53.34" y="135.509" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="SW-OUT" gate="-2" x="53.34" y="129.54" smashed="yes" grouprefs="SWITCH GAS">
<attribute name="VALUE" x="50.8" y="125.857" size="1.778" layer="96"/>
<attribute name="NAME" x="53.34" y="130.429" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="TPS" gate="-1" x="22.86" y="88.9" smashed="yes" grouprefs="TPS-MB GAS">
<attribute name="NAME" x="22.86" y="89.789" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="TPS" gate="-2" x="22.86" y="83.82" smashed="yes" grouprefs="TPS-MB GAS">
<attribute name="NAME" x="22.86" y="84.709" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="TPS" gate="-3" x="22.86" y="78.74" smashed="yes" grouprefs="TPS-MB GAS">
<attribute name="VALUE" x="20.32" y="75.057" size="1.778" layer="96"/>
<attribute name="NAME" x="22.86" y="79.629" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="MB" gate="-1" x="22.86" y="68.58" smashed="yes" grouprefs="TPS-MB GAS">
<attribute name="NAME" x="22.86" y="69.469" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="MB" gate="-2" x="22.86" y="63.5" smashed="yes" grouprefs="TPS-MB GAS">
<attribute name="NAME" x="22.86" y="64.389" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="MB" gate="-3" x="22.86" y="58.42" smashed="yes" grouprefs="TPS-MB GAS">
<attribute name="VALUE" x="20.32" y="54.737" size="1.778" layer="96"/>
<attribute name="NAME" x="22.86" y="59.309" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="DRV-STEPPER" gate="-1" x="109.22" y="139.7" smashed="yes" grouprefs="STIR">
<attribute name="NAME" x="109.22" y="140.589" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="DRV-STEPPER" gate="-2" x="109.22" y="134.62" smashed="yes" grouprefs="STIR">
<attribute name="NAME" x="109.22" y="135.509" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="DRV-STEPPER" gate="-3" x="109.22" y="129.54" smashed="yes" grouprefs="STIR">
<attribute name="NAME" x="109.22" y="130.429" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="DRV-STEPPER" gate="-4" x="109.22" y="124.46" smashed="yes" grouprefs="STIR">
<attribute name="VALUE" x="106.68" y="120.777" size="1.778" layer="96"/>
<attribute name="NAME" x="109.22" y="125.349" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="REM-DRV" gate="G$1" x="88.9" y="81.28" smashed="yes" grouprefs="REM">
<attribute name="VALUE" x="87.63" y="68.58" size="1.778" layer="96"/>
<attribute name="NAME" x="87.63" y="92.202" size="1.778" layer="95"/>
</instance>
<instance part="24V" gate="-1" x="124.46" y="88.9" smashed="yes" grouprefs="REM">
<attribute name="NAME" x="124.46" y="89.789" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="24V" gate="-2" x="124.46" y="83.82" smashed="yes" grouprefs="REM">
<attribute name="VALUE" x="121.92" y="80.137" size="1.778" layer="96"/>
<attribute name="NAME" x="124.46" y="84.709" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="REM-MTR" gate="-1" x="124.46" y="78.74" smashed="yes" grouprefs="REM">
<attribute name="NAME" x="124.46" y="79.629" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="REM-MTR" gate="-2" x="124.46" y="73.66" smashed="yes" grouprefs="REM">
<attribute name="VALUE" x="121.92" y="69.977" size="1.778" layer="96"/>
<attribute name="NAME" x="124.46" y="74.549" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="REM-KTC" gate="-1" x="101.6" y="66.04" smashed="yes" grouprefs="REM">
<attribute name="NAME" x="101.6" y="66.929" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="REM-KTC" gate="-2" x="101.6" y="60.96" smashed="yes" grouprefs="REM">
<attribute name="NAME" x="101.6" y="61.849" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="REM-KTC" gate="-3" x="101.6" y="55.88" smashed="yes" grouprefs="REM">
<attribute name="VALUE" x="99.06" y="52.197" size="1.778" layer="96"/>
<attribute name="NAME" x="101.6" y="56.769" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="ENC" gate="-1" x="132.08" y="132.08" smashed="yes" rot="R90" grouprefs="STIR">
<attribute name="NAME" x="132.842" y="134.62" size="1.524" layer="95" rot="R90"/>
<attribute name="VALUE" x="130.683" y="131.318" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="ENC" gate="-2" x="134.62" y="132.08" smashed="yes" rot="R90" grouprefs="STIR">
<attribute name="NAME" x="135.382" y="134.62" size="1.524" layer="95" rot="R90"/>
</instance>
<instance part="ENC" gate="-3" x="137.16" y="132.08" smashed="yes" rot="R90" grouprefs="STIR">
<attribute name="NAME" x="137.922" y="134.62" size="1.524" layer="95" rot="R90"/>
</instance>
<instance part="ENC" gate="-4" x="139.7" y="132.08" smashed="yes" rot="R90" grouprefs="STIR">
<attribute name="NAME" x="140.462" y="134.62" size="1.524" layer="95" rot="R90"/>
</instance>
<instance part="ENC" gate="-5" x="142.24" y="132.08" smashed="yes" rot="R90" grouprefs="STIR">
<attribute name="NAME" x="143.002" y="134.62" size="1.524" layer="95" rot="R90"/>
</instance>
<instance part="ENC" gate="-6" x="144.78" y="132.08" smashed="yes" rot="R90" grouprefs="STIR">
<attribute name="NAME" x="145.542" y="134.62" size="1.524" layer="95" rot="R90"/>
</instance>
<instance part="ENC" gate="-7" x="147.32" y="132.08" smashed="yes" rot="R90" grouprefs="STIR">
<attribute name="NAME" x="148.082" y="134.62" size="1.524" layer="95" rot="R90"/>
</instance>
<instance part="ENC" gate="-8" x="149.86" y="132.08" smashed="yes" rot="R90" grouprefs="STIR">
<attribute name="NAME" x="150.622" y="134.62" size="1.524" layer="95" rot="R90"/>
</instance>
<instance part="ENC" gate="-9" x="152.4" y="132.08" smashed="yes" rot="R90" grouprefs="STIR">
<attribute name="NAME" x="153.162" y="134.62" size="1.524" layer="95" rot="R90"/>
</instance>
<instance part="ENC" gate="-10" x="154.94" y="132.08" smashed="yes" rot="R90" grouprefs="STIR">
<attribute name="NAME" x="155.702" y="134.62" size="1.524" layer="95" rot="R90"/>
</instance>
<instance part="12V" gate="-1" x="109.22" y="114.3" smashed="yes" grouprefs="STIR">
<attribute name="NAME" x="109.22" y="115.189" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="12V" gate="-2" x="109.22" y="109.22" smashed="yes" grouprefs="STIR">
<attribute name="VALUE" x="106.68" y="105.537" size="1.778" layer="96"/>
<attribute name="NAME" x="109.22" y="110.109" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="ENC2" gate="-1" x="139.7" y="109.22" smashed="yes" rot="R180" grouprefs="STIR">
<attribute name="NAME" x="137.16" y="109.982" size="1.524" layer="95" rot="R180"/>
<attribute name="VALUE" x="140.462" y="107.823" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="ENC2" gate="-2" x="139.7" y="114.3" smashed="yes" rot="R180" grouprefs="STIR">
<attribute name="NAME" x="137.16" y="115.062" size="1.524" layer="95" rot="R180"/>
<attribute name="VALUE" x="140.462" y="112.903" size="1.778" layer="96" rot="R180"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="DAC" class="0">
<segment>
<pinref part="DAC" gate="G$1" pin="6"/>
<wire x1="60.96" y1="91.44" x2="63.5" y2="91.44" width="0.1524" layer="91" grouprefs="GAS"/>
<label x="63.5" y="91.44" size="1.4224" layer="95" xref="yes" grouprefs="GAS"/>
</segment>
</net>
<net name="GND" class="0">
<segment>
<pinref part="DAC" gate="G$1" pin="5"/>
<wire x1="60.96" y1="88.9" x2="63.5" y2="88.9" width="0.1524" layer="91" grouprefs="GAS"/>
<label x="63.5" y="88.9" size="1.4224" layer="95" xref="yes" grouprefs="GAS"/>
</segment>
<segment>
<pinref part="DAC" gate="G$1" pin="1"/>
<wire x1="60.96" y1="78.74" x2="63.5" y2="78.74" width="0.1524" layer="91" grouprefs="GAS"/>
<label x="63.5" y="78.74" size="1.4224" layer="95" xref="yes" grouprefs="GAS"/>
</segment>
<segment>
<pinref part="RELAY-1" gate="G$1" pin="2"/>
<wire x1="22.86" y1="109.22" x2="25.4" y2="109.22" width="0.1524" layer="91" grouprefs="RELAY GAS"/>
<label x="25.4" y="109.22" size="1.4224" layer="95" xref="yes" grouprefs="RELAY GAS"/>
</segment>
<segment>
<pinref part="SW-IN" gate="-1" pin="KL"/>
<wire x1="30.48" y1="134.62" x2="33.02" y2="134.62" width="0.1524" layer="91" grouprefs="SWITCH GAS"/>
<label x="33.02" y="134.62" size="1.4224" layer="95" xref="yes" grouprefs="SWITCH GAS"/>
</segment>
<segment>
<pinref part="TPS" gate="-2" pin="KL"/>
<wire x1="27.94" y1="83.82" x2="33.02" y2="83.82" width="0.1524" layer="91" grouprefs="TPS-MB GAS"/>
<label x="33.02" y="83.82" size="1.4224" layer="95" xref="yes" grouprefs="TPS-MB GAS"/>
</segment>
<segment>
<pinref part="MB" gate="-2" pin="KL"/>
<wire x1="27.94" y1="63.5" x2="33.02" y2="63.5" width="0.1524" layer="91" grouprefs="TPS-MB GAS"/>
<label x="33.02" y="63.5" size="1.4224" layer="95" xref="yes" grouprefs="TPS-MB GAS"/>
</segment>
<segment>
<pinref part="REM-DRV" gate="G$1" pin="6"/>
<wire x1="96.52" y1="86.36" x2="99.06" y2="86.36" width="0.1524" layer="91" grouprefs="REM"/>
<label x="99.06" y="86.36" size="1.4224" layer="95" xref="yes" grouprefs="REM"/>
</segment>
<segment>
<pinref part="REM-KTC" gate="-3" pin="KL"/>
<wire x1="106.68" y1="55.88" x2="109.22" y2="55.88" width="0.1524" layer="91" grouprefs="REM"/>
<label x="109.22" y="55.88" size="1.4224" layer="95" xref="yes" grouprefs="REM"/>
</segment>
</net>
<net name="SCL" class="0">
<segment>
<pinref part="DAC" gate="G$1" pin="4"/>
<wire x1="60.96" y1="86.36" x2="63.5" y2="86.36" width="0.1524" layer="91" grouprefs="GAS"/>
<label x="63.5" y="86.36" size="1.4224" layer="95" xref="yes" grouprefs="GAS"/>
</segment>
</net>
<net name="SDA" class="0">
<segment>
<pinref part="DAC" gate="G$1" pin="3"/>
<wire x1="60.96" y1="83.82" x2="63.5" y2="83.82" width="0.1524" layer="91" grouprefs="GAS"/>
<label x="63.5" y="83.82" size="1.4224" layer="95" xref="yes" grouprefs="GAS"/>
</segment>
</net>
<net name="VCC" class="0">
<segment>
<pinref part="DAC" gate="G$1" pin="2"/>
<wire x1="60.96" y1="81.28" x2="63.5" y2="81.28" width="0.1524" layer="91" grouprefs="GAS"/>
<label x="63.5" y="81.28" size="1.4224" layer="95" xref="yes" grouprefs="GAS"/>
</segment>
<segment>
<pinref part="SW-IN" gate="-2" pin="KL"/>
<wire x1="30.48" y1="129.54" x2="33.02" y2="129.54" width="0.1524" layer="91" grouprefs="SWITCH GAS"/>
<label x="33.02" y="129.54" size="1.4224" layer="95" xref="yes" grouprefs="SWITCH GAS"/>
</segment>
<segment>
<pinref part="DRV-STEPPER" gate="-1" pin="KL"/>
<wire x1="114.3" y1="139.7" x2="116.84" y2="139.7" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="116.84" y="139.7" size="1.4224" layer="95" xref="yes" grouprefs="STIR"/>
</segment>
<segment>
<pinref part="REM-DRV" gate="G$1" pin="7"/>
<wire x1="96.52" y1="88.9" x2="99.06" y2="88.9" width="0.1524" layer="91" grouprefs="REM"/>
<label x="99.06" y="88.9" size="1.4224" layer="95" xref="yes" grouprefs="REM"/>
</segment>
<segment>
<pinref part="REM-KTC" gate="-1" pin="KL"/>
<wire x1="106.68" y1="66.04" x2="109.22" y2="66.04" width="0.1524" layer="91" grouprefs="REM"/>
<label x="109.22" y="66.04" size="1.4224" layer="95" xref="yes" grouprefs="REM"/>
</segment>
</net>
<net name="IN-RELAY" class="0">
<segment>
<pinref part="RELAY-1" gate="G$1" pin="1"/>
<wire x1="22.86" y1="106.68" x2="25.4" y2="106.68" width="0.1524" layer="91" grouprefs="RELAY GAS"/>
<label x="25.4" y="106.68" size="1.4224" layer="95" xref="yes" grouprefs="RELAY GAS"/>
</segment>
<segment>
<pinref part="SW-OUT" gate="-2" pin="KL"/>
<wire x1="58.42" y1="129.54" x2="60.96" y2="129.54" width="0.1524" layer="91" grouprefs="SWITCH GAS"/>
<label x="60.96" y="129.54" size="1.4224" layer="95" xref="yes" grouprefs="SWITCH GAS"/>
<pinref part="SW-OUT" gate="-1" pin="KL"/>
<wire x1="58.42" y1="134.62" x2="60.96" y2="134.62" width="0.1524" layer="91" grouprefs="SWITCH GAS"/>
<label x="60.96" y="134.62" size="1.4224" layer="95" xref="yes" grouprefs="SWITCH GAS"/>
<wire x1="58.42" y1="129.54" x2="58.42" y2="134.62" width="0.1524" layer="91" grouprefs="SWITCH GAS"/>
<junction x="58.42" y="129.54" grouprefs="SWITCH GAS"/>
<junction x="58.42" y="134.62" grouprefs="SWITCH GAS"/>
</segment>
</net>
<net name="NC" class="0">
<segment>
<pinref part="RELAY-2" gate="-1" pin="KL"/>
<wire x1="55.88" y1="114.3" x2="58.42" y2="114.3" width="0.1524" layer="91" grouprefs="RELAY GAS"/>
<label x="58.42" y="114.3" size="1.4224" layer="95" xref="yes" grouprefs="RELAY GAS"/>
</segment>
<segment>
<pinref part="TPS" gate="-3" pin="KL"/>
<wire x1="27.94" y1="78.74" x2="33.02" y2="78.74" width="0.1524" layer="91" grouprefs="TPS-MB GAS"/>
<label x="33.02" y="78.74" size="1.4224" layer="95" xref="yes" grouprefs="TPS-MB GAS"/>
</segment>
</net>
<net name="COM" class="0">
<segment>
<pinref part="RELAY-2" gate="-2" pin="KL"/>
<wire x1="55.88" y1="109.22" x2="58.42" y2="109.22" width="0.1524" layer="91" grouprefs="RELAY GAS"/>
<label x="58.42" y="109.22" size="1.4224" layer="95" xref="yes" grouprefs="RELAY GAS"/>
</segment>
<segment>
<pinref part="MB" gate="-3" pin="KL"/>
<wire x1="27.94" y1="58.42" x2="33.02" y2="58.42" width="0.1524" layer="91" grouprefs="TPS-MB GAS"/>
<label x="33.02" y="58.42" size="1.4224" layer="95" xref="yes" grouprefs="TPS-MB GAS"/>
</segment>
</net>
<net name="NO" class="0">
<segment>
<pinref part="RELAY-2" gate="-3" pin="KL"/>
<wire x1="55.88" y1="104.14" x2="58.42" y2="104.14" width="0.1524" layer="91" grouprefs="RELAY GAS"/>
<label x="58.42" y="104.14" size="1.4224" layer="95" xref="yes" grouprefs="RELAY GAS"/>
</segment>
</net>
<net name="VCC2" class="0">
<segment>
<pinref part="MB" gate="-1" pin="KL"/>
<wire x1="27.94" y1="68.58" x2="33.02" y2="68.58" width="0.1524" layer="91" grouprefs="TPS-MB GAS"/>
<label x="33.02" y="68.58" size="1.4224" layer="95" xref="yes" grouprefs="TPS-MB GAS"/>
</segment>
<segment>
<pinref part="TPS" gate="-1" pin="KL"/>
<wire x1="27.94" y1="88.9" x2="33.02" y2="88.9" width="0.1524" layer="91" grouprefs="TPS-MB GAS"/>
<label x="33.02" y="88.9" size="1.4224" layer="95" xref="yes" grouprefs="TPS-MB GAS"/>
</segment>
<segment>
<pinref part="RELAY-1" gate="G$1" pin="3"/>
<wire x1="22.86" y1="111.76" x2="25.4" y2="111.76" width="0.1524" layer="91" grouprefs="RELAY GAS"/>
<label x="25.4" y="111.76" size="1.4224" layer="95" xref="yes" grouprefs="RELAY GAS"/>
</segment>
</net>
<net name="EN-" class="0">
<segment>
<pinref part="DRV-STEPPER" gate="-2" pin="KL"/>
<wire x1="114.3" y1="134.62" x2="116.84" y2="134.62" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="116.84" y="134.62" size="1.4224" layer="95" xref="yes" grouprefs="STIR"/>
</segment>
</net>
<net name="DIR-" class="0">
<segment>
<pinref part="DRV-STEPPER" gate="-3" pin="KL"/>
<wire x1="114.3" y1="129.54" x2="116.84" y2="129.54" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="116.84" y="129.54" size="1.4224" layer="95" xref="yes" grouprefs="STIR"/>
</segment>
</net>
<net name="PUL-" class="0">
<segment>
<pinref part="DRV-STEPPER" gate="-4" pin="KL"/>
<wire x1="114.3" y1="124.46" x2="116.84" y2="124.46" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="116.84" y="124.46" size="1.4224" layer="95" xref="yes" grouprefs="STIR"/>
</segment>
</net>
<net name="EN" class="0">
<segment>
<pinref part="REM-DRV" gate="G$1" pin="5"/>
<wire x1="96.52" y1="83.82" x2="99.06" y2="83.82" width="0.1524" layer="91" grouprefs="REM"/>
<label x="99.06" y="83.82" size="1.4224" layer="95" xref="yes" grouprefs="REM"/>
</segment>
</net>
<net name="CS" class="0">
<segment>
<pinref part="REM-DRV" gate="G$1" pin="4"/>
<wire x1="96.52" y1="81.28" x2="99.06" y2="81.28" width="0.1524" layer="91" grouprefs="REM"/>
<label x="99.06" y="81.28" size="1.4224" layer="95" xref="yes" grouprefs="REM"/>
</segment>
</net>
<net name="INA" class="0">
<segment>
<pinref part="REM-DRV" gate="G$1" pin="3"/>
<wire x1="96.52" y1="78.74" x2="99.06" y2="78.74" width="0.1524" layer="91" grouprefs="REM"/>
<label x="99.06" y="78.74" size="1.4224" layer="95" xref="yes" grouprefs="REM"/>
</segment>
</net>
<net name="PWM" class="0">
<segment>
<pinref part="REM-DRV" gate="G$1" pin="1"/>
<wire x1="96.52" y1="73.66" x2="99.06" y2="73.66" width="0.1524" layer="91" grouprefs="REM"/>
<label x="99.06" y="73.66" size="1.4224" layer="95" xref="yes" grouprefs="REM"/>
</segment>
</net>
<net name="INB" class="0">
<segment>
<pinref part="REM-DRV" gate="G$1" pin="2"/>
<wire x1="96.52" y1="76.2" x2="99.06" y2="76.2" width="0.1524" layer="91" grouprefs="REM"/>
<label x="99.06" y="76.2" size="1.4224" layer="95" xref="yes" grouprefs="REM"/>
</segment>
</net>
<net name="MTR-" class="0">
<segment>
<pinref part="24V" gate="-2" pin="KL"/>
<wire x1="129.54" y1="83.82" x2="132.08" y2="83.82" width="0.1524" layer="91" grouprefs="REM"/>
<label x="132.08" y="83.82" size="1.4224" layer="95" xref="yes" grouprefs="REM"/>
</segment>
<segment>
<pinref part="REM-MTR" gate="-2" pin="KL"/>
<wire x1="129.54" y1="73.66" x2="132.08" y2="73.66" width="0.1524" layer="91" grouprefs="REM"/>
<label x="132.08" y="73.66" size="1.4224" layer="95" xref="yes" grouprefs="REM"/>
</segment>
</net>
<net name="MTR+" class="0">
<segment>
<pinref part="24V" gate="-1" pin="KL"/>
<wire x1="129.54" y1="88.9" x2="132.08" y2="88.9" width="0.1524" layer="91" grouprefs="REM"/>
<label x="132.08" y="88.9" size="1.4224" layer="95" xref="yes" grouprefs="REM"/>
</segment>
<segment>
<pinref part="REM-MTR" gate="-1" pin="KL"/>
<wire x1="129.54" y1="78.74" x2="132.08" y2="78.74" width="0.1524" layer="91" grouprefs="REM"/>
<label x="132.08" y="78.74" size="1.4224" layer="95" xref="yes" grouprefs="REM"/>
</segment>
</net>
<net name="KTC" class="0">
<segment>
<pinref part="REM-KTC" gate="-2" pin="KL"/>
<wire x1="106.68" y1="60.96" x2="109.22" y2="60.96" width="0.1524" layer="91" grouprefs="REM"/>
<label x="109.22" y="60.96" size="1.4224" layer="95" xref="yes" grouprefs="REM"/>
</segment>
</net>
<net name="ENC1" class="0">
<segment>
<pinref part="ENC" gate="-1" pin="S"/>
<wire x1="132.08" y1="129.54" x2="132.08" y2="127" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="132.08" y="127" size="1.4224" layer="95" rot="R270" xref="yes" grouprefs="STIR"/>
</segment>
</net>
<net name="ENC2" class="0">
<segment>
<pinref part="ENC" gate="-2" pin="S"/>
<wire x1="134.62" y1="129.54" x2="134.62" y2="127" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="134.62" y="127" size="1.4224" layer="95" rot="R270" xref="yes" grouprefs="STIR"/>
</segment>
</net>
<net name="ENC3" class="0">
<segment>
<pinref part="ENC" gate="-3" pin="S"/>
<wire x1="137.16" y1="129.54" x2="137.16" y2="127" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="137.16" y="127" size="1.4224" layer="95" rot="R270" xref="yes" grouprefs="STIR"/>
</segment>
</net>
<net name="ENC4" class="0">
<segment>
<pinref part="ENC" gate="-4" pin="S"/>
<wire x1="139.7" y1="129.54" x2="139.7" y2="127" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="139.7" y="127" size="1.4224" layer="95" rot="R270" xref="yes" grouprefs="STIR"/>
</segment>
</net>
<net name="ENC5" class="0">
<segment>
<pinref part="ENC" gate="-5" pin="S"/>
<wire x1="142.24" y1="129.54" x2="142.24" y2="127" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="142.24" y="127" size="1.4224" layer="95" rot="R270" xref="yes" grouprefs="STIR"/>
</segment>
</net>
<net name="ENC6" class="0">
<segment>
<pinref part="ENC" gate="-6" pin="S"/>
<wire x1="144.78" y1="129.54" x2="144.78" y2="127" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="144.78" y="127" size="1.4224" layer="95" rot="R270" xref="yes" grouprefs="STIR"/>
</segment>
</net>
<net name="ENC7" class="0">
<segment>
<pinref part="ENC" gate="-7" pin="S"/>
<wire x1="147.32" y1="129.54" x2="147.32" y2="127" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="147.32" y="127" size="1.4224" layer="95" rot="R270" xref="yes" grouprefs="STIR"/>
</segment>
</net>
<net name="ENC8" class="0">
<segment>
<pinref part="ENC" gate="-8" pin="S"/>
<wire x1="149.86" y1="129.54" x2="149.86" y2="127" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="149.86" y="127" size="1.4224" layer="95" rot="R270" xref="yes" grouprefs="STIR"/>
</segment>
</net>
<net name="ENC9" class="0">
<segment>
<pinref part="ENC" gate="-9" pin="S"/>
<wire x1="152.4" y1="129.54" x2="152.4" y2="127" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="152.4" y="127" size="1.4224" layer="95" rot="R270" xref="yes" grouprefs="STIR"/>
</segment>
</net>
<net name="ENC10" class="0">
<segment>
<pinref part="ENC" gate="-10" pin="S"/>
<wire x1="154.94" y1="129.54" x2="154.94" y2="127" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="154.94" y="127" size="1.4224" layer="95" rot="R270" xref="yes" grouprefs="STIR"/>
</segment>
</net>
<net name="ENC+" class="0">
<segment>
<pinref part="12V" gate="-2" pin="KL"/>
<wire x1="114.3" y1="109.22" x2="116.84" y2="109.22" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="116.84" y="109.22" size="1.4224" layer="95" xref="yes" grouprefs="STIR"/>
</segment>
<segment>
<pinref part="ENC2" gate="-1" pin="S"/>
<wire x1="142.24" y1="109.22" x2="144.78" y2="109.22" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="144.78" y="109.22" size="1.4224" layer="95" xref="yes" grouprefs="STIR"/>
</segment>
</net>
<net name="ENC-" class="0">
<segment>
<pinref part="12V" gate="-1" pin="KL"/>
<wire x1="114.3" y1="114.3" x2="116.84" y2="114.3" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="116.84" y="114.3" size="1.4224" layer="95" xref="yes" grouprefs="STIR"/>
</segment>
<segment>
<pinref part="ENC2" gate="-2" pin="S"/>
<wire x1="142.24" y1="114.3" x2="144.78" y2="114.3" width="0.1524" layer="91" grouprefs="STIR"/>
<label x="144.78" y="114.3" size="1.4224" layer="95" xref="yes" grouprefs="STIR"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports the association of 3D packages
with devices in libraries, schematics, and board files. Those 3D
packages will not be understood (or retained) with this version.
</note>
<note version="9.5" severity="warning">
Since Version 9.5, EAGLE supports persistent groups with
schematics, and board files. Those persistent groups
will not be understood (or retained) with this version.
</note>
</compatibility>
</eagle>
