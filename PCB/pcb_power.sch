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
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
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
<layer number="51" name="tDocu" color="6" fill="1" visible="no" active="no"/>
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
<library name="solpad" urn="urn:adsk.eagle:library:364">
<description>&lt;b&gt;Solder Pads/Test Points&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="SE14" urn="urn:adsk.eagle:footprint:26500/1" library_version="2">
<description>&lt;b&gt;SOLDER PAD&lt;/b&gt;&lt;p&gt;
drill 1.4 mm</description>
<wire x1="-1.524" y1="0.635" x2="-1.524" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-1.524" x2="1.524" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="0.635" y1="1.524" x2="1.524" y2="0.635" width="0.1524" layer="21"/>
<wire x1="1.524" y1="0.635" x2="1.524" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="-1.524" y1="0.635" x2="-0.635" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="1.524" x2="0.635" y2="1.524" width="0.1524" layer="21"/>
<wire x1="-0.635" y1="-1.524" x2="-1.524" y2="-0.635" width="0.1524" layer="21"/>
<wire x1="0.635" y1="-1.524" x2="-0.635" y2="-1.524" width="0.1524" layer="21"/>
<circle x="0" y="0" radius="0.762" width="0.1524" layer="51"/>
<circle x="0" y="0" radius="0.381" width="0.254" layer="51"/>
<pad name="MP" x="0" y="0" drill="1.397" diameter="2.54" shape="octagon"/>
<text x="-1.397" y="1.778" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="0" y="0.381" size="0.0254" layer="27">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="SE14" urn="urn:adsk.eagle:package:26504/1" type="box" library_version="2">
<description>SOLDER PAD
drill 1.4 mm</description>
<packageinstances>
<packageinstance name="SE14"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="LSP" urn="urn:adsk.eagle:symbol:26492/1" library_version="2">
<wire x1="-1.016" y1="2.032" x2="1.016" y2="0" width="0.254" layer="94"/>
<wire x1="-1.016" y1="0" x2="1.016" y2="2.032" width="0.254" layer="94"/>
<circle x="0" y="1.016" radius="1.016" width="0.4064" layer="94"/>
<text x="-1.27" y="2.921" size="1.778" layer="95">&gt;NAME</text>
<pin name="MP" x="0" y="-2.54" visible="off" length="short" direction="pas" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="SE14" urn="urn:adsk.eagle:component:26510/2" prefix="LSP" library_version="2">
<description>&lt;b&gt;SOLDER PAD&lt;/b&gt;&lt;p&gt; E1553,  drill 1,4mm, distributor Buerklin, 07F820</description>
<gates>
<gate name="1" symbol="LSP" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SE14">
<connects>
<connect gate="1" pin="MP" pad="MP"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:26504/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="POPULARITY" value="9" constant="no"/>
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
</packages>
<packages3d>
<package3d name="W237-102" urn="urn:adsk.eagle:package:10688/1" type="box" library_version="2">
<description>WAGO SCREW CLAMP</description>
<packageinstances>
<packageinstance name="W237-102"/>
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
<class number="1" name="PWR" width="2.54" drill="0">
<clearance class="1" value="0.4064"/>
</class>
<class number="2" name="GND" width="2.54" drill="0">
<clearance class="2" value="0.8128"/>
</class>
</classes>
<parts>
<part name="BATT" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="EMGS" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="ENC" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="STEPPER" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="REM" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="JPR1_" library="solpad" library_urn="urn:adsk.eagle:library:364" deviceset="SE14" device="" package3d_urn="urn:adsk.eagle:package:26504/1"/>
<part name="JPR1" library="solpad" library_urn="urn:adsk.eagle:library:364" deviceset="SE14" device="" package3d_urn="urn:adsk.eagle:package:26504/1"/>
<part name="ADD" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="ADD2-" library="solpad" library_urn="urn:adsk.eagle:library:364" deviceset="SE14" device="" package3d_urn="urn:adsk.eagle:package:26504/1"/>
<part name="ADD2+" library="solpad" library_urn="urn:adsk.eagle:library:364" deviceset="SE14" device="" package3d_urn="urn:adsk.eagle:package:26504/1"/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="BATT" gate="-1" x="25.4" y="71.12" smashed="yes">
<attribute name="NAME" x="24.13" y="72.009" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="BATT" gate="-2" x="25.4" y="66.04" smashed="yes">
<attribute name="VALUE" x="25.4" y="68.58" size="1.778" layer="96"/>
<attribute name="NAME" x="24.13" y="66.929" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="EMGS" gate="-1" x="25.4" y="50.8" smashed="yes">
<attribute name="NAME" x="24.13" y="51.689" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="EMGS" gate="-2" x="25.4" y="45.72" smashed="yes">
<attribute name="VALUE" x="25.4" y="48.26" size="1.778" layer="96"/>
<attribute name="NAME" x="24.13" y="46.609" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="ENC" gate="-1" x="55.88" y="81.28" smashed="yes" rot="R270">
<attribute name="NAME" x="56.769" y="82.55" size="1.778" layer="95" rot="R90"/>
</instance>
<instance part="ENC" gate="-2" x="50.8" y="81.28" smashed="yes" rot="R270">
<attribute name="VALUE" x="53.34" y="81.28" size="1.778" layer="96" rot="R270"/>
<attribute name="NAME" x="51.689" y="82.55" size="1.778" layer="95" rot="R90"/>
</instance>
<instance part="STEPPER" gate="-1" x="83.82" y="55.88" smashed="yes" rot="R180">
<attribute name="NAME" x="85.09" y="54.991" size="1.778" layer="95"/>
</instance>
<instance part="STEPPER" gate="-2" x="83.82" y="60.96" smashed="yes" rot="R180">
<attribute name="VALUE" x="83.82" y="58.42" size="1.778" layer="96" rot="R180"/>
<attribute name="NAME" x="85.09" y="60.071" size="1.778" layer="95"/>
</instance>
<instance part="REM" gate="-1" x="48.26" y="35.56" smashed="yes" rot="R90">
<attribute name="NAME" x="47.371" y="34.29" size="1.778" layer="95" rot="R270"/>
</instance>
<instance part="REM" gate="-2" x="53.34" y="35.56" smashed="yes" rot="R90">
<attribute name="VALUE" x="50.8" y="35.56" size="1.778" layer="96" rot="R90"/>
<attribute name="NAME" x="52.451" y="34.29" size="1.778" layer="95" rot="R270"/>
</instance>
<instance part="JPR1_" gate="1" x="38.1" y="40.64" smashed="yes" rot="R180">
<attribute name="NAME" x="39.37" y="37.719" size="1.778" layer="95" rot="R180"/>
</instance>
<instance part="JPR1" gate="1" x="40.64" y="50.8" smashed="yes" rot="R270">
<attribute name="NAME" x="43.561" y="52.07" size="1.778" layer="95" rot="R270"/>
</instance>
<instance part="ADD" gate="-1" x="86.36" y="71.12" smashed="yes" rot="R180">
<attribute name="NAME" x="86.36" y="70.231" size="1.778" layer="95"/>
</instance>
<instance part="ADD" gate="-2" x="86.36" y="76.2" smashed="yes" rot="R180">
<attribute name="VALUE" x="88.9" y="79.883" size="1.778" layer="96" rot="R180"/>
<attribute name="NAME" x="86.36" y="75.311" size="1.778" layer="95"/>
</instance>
<instance part="ADD2-" gate="1" x="73.66" y="78.74" smashed="yes">
<attribute name="NAME" x="72.39" y="81.661" size="1.778" layer="95"/>
</instance>
<instance part="ADD2+" gate="1" x="78.74" y="66.04" smashed="yes" rot="R270">
<attribute name="NAME" x="81.661" y="67.31" size="1.778" layer="95" rot="R270"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="IN" class="1">
<segment>
<pinref part="BATT" gate="-1" pin="KL"/>
<wire x1="30.48" y1="71.12" x2="35.56" y2="71.12" width="0.1524" layer="91"/>
<pinref part="EMGS" gate="-1" pin="KL"/>
<wire x1="35.56" y1="71.12" x2="35.56" y2="50.8" width="0.1524" layer="91"/>
<wire x1="35.56" y1="50.8" x2="30.48" y2="50.8" width="0.1524" layer="91"/>
<pinref part="JPR1" gate="1" pin="MP"/>
<wire x1="35.56" y1="50.8" x2="38.1" y2="50.8" width="0.1524" layer="91"/>
<junction x="35.56" y="50.8"/>
<label x="35.56" y="71.12" size="1.778" layer="95" xref="yes"/>
</segment>
</net>
<net name="24V" class="1">
<segment>
<pinref part="EMGS" gate="-2" pin="KL"/>
<wire x1="30.48" y1="45.72" x2="38.1" y2="45.72" width="0.1524" layer="91"/>
<pinref part="JPR1_" gate="1" pin="MP"/>
<wire x1="38.1" y1="45.72" x2="38.1" y2="43.18" width="0.1524" layer="91"/>
<pinref part="REM" gate="-1" pin="KL"/>
<wire x1="38.1" y1="45.72" x2="48.26" y2="45.72" width="0.1524" layer="91"/>
<wire x1="48.26" y1="45.72" x2="48.26" y2="40.64" width="0.1524" layer="91"/>
<junction x="38.1" y="45.72"/>
<wire x1="48.26" y1="45.72" x2="48.26" y2="55.88" width="0.1524" layer="91"/>
<junction x="48.26" y="45.72"/>
<pinref part="STEPPER" gate="-1" pin="KL"/>
<wire x1="48.26" y1="55.88" x2="55.88" y2="55.88" width="0.1524" layer="91"/>
<label x="66.04" y="55.88" size="1.778" layer="95" rot="R270" xref="yes"/>
<pinref part="ENC" gate="-1" pin="KL"/>
<wire x1="55.88" y1="55.88" x2="76.2" y2="55.88" width="0.1524" layer="91"/>
<wire x1="76.2" y1="55.88" x2="78.74" y2="55.88" width="0.1524" layer="91"/>
<wire x1="55.88" y1="76.2" x2="55.88" y2="55.88" width="0.1524" layer="91"/>
<junction x="55.88" y="55.88"/>
<pinref part="ADD" gate="-1" pin="KL"/>
<wire x1="81.28" y1="71.12" x2="76.2" y2="71.12" width="0.1524" layer="91"/>
<wire x1="76.2" y1="71.12" x2="76.2" y2="66.04" width="0.1524" layer="91"/>
<junction x="76.2" y="55.88"/>
<pinref part="ADD2+" gate="1" pin="MP"/>
<wire x1="76.2" y1="66.04" x2="76.2" y2="55.88" width="0.1524" layer="91"/>
<junction x="76.2" y="66.04"/>
</segment>
</net>
<net name="GND" class="2">
<segment>
<pinref part="STEPPER" gate="-2" pin="KL"/>
<wire x1="78.74" y1="60.96" x2="73.66" y2="60.96" width="0.1524" layer="91"/>
<wire x1="73.66" y1="60.96" x2="53.34" y2="60.96" width="0.1524" layer="91"/>
<wire x1="53.34" y1="60.96" x2="50.8" y2="60.96" width="0.1524" layer="91"/>
<wire x1="50.8" y1="60.96" x2="50.8" y2="66.04" width="0.1524" layer="91"/>
<wire x1="50.8" y1="66.04" x2="50.8" y2="76.2" width="0.1524" layer="91"/>
<pinref part="ENC" gate="-2" pin="KL"/>
<pinref part="BATT" gate="-2" pin="KL"/>
<wire x1="30.48" y1="66.04" x2="50.8" y2="66.04" width="0.1524" layer="91"/>
<junction x="50.8" y="66.04"/>
<pinref part="REM" gate="-2" pin="KL"/>
<wire x1="53.34" y1="40.64" x2="53.34" y2="60.96" width="0.1524" layer="91"/>
<junction x="53.34" y="60.96"/>
<label x="66.04" y="60.96" size="1.778" layer="95" rot="R90" xref="yes"/>
<wire x1="73.66" y1="60.96" x2="73.66" y2="76.2" width="0.1524" layer="91"/>
<junction x="73.66" y="60.96"/>
<pinref part="ADD" gate="-2" pin="KL"/>
<wire x1="73.66" y1="76.2" x2="81.28" y2="76.2" width="0.1524" layer="91"/>
<pinref part="ADD2-" gate="1" pin="MP"/>
<junction x="73.66" y="76.2"/>
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
</compatibility>
</eagle>
