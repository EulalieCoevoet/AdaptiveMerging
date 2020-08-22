package glProfileCheck;

import com.jogamp.opengl.GLProfile;

public class CheckGLProfiles {
	 public static void main(String[] args) {
		 GLProfile glprofile = GLProfile.getDefault();
		 System.out.println( "Default GLProfile is " + glprofile );
		 for ( String s : GLProfile.GL_PROFILE_LIST_ALL ) {
			 try {
				 String[] glps = new String[] { s };
				 GLProfile glp = GLProfile.get( glps, true );
				 System.out.println( "Requested " + s + " Got " + glp );
			 } catch (Exception e ) {
				 System.out.println( "Requested " + s + " Got GLException " );
			 }
		 }
	 }
}
