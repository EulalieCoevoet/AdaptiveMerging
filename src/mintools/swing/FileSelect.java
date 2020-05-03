package mintools.swing;

import java.io.File;
import javax.swing.*;
import javax.swing.filechooser.*;

/**
 * Helper class for file loading dialogs
 */
public class FileSelect {

    /**
     * Open a file chooser to select a file.
     * 
     * Returns a file, or null if cancel is selected.
     * 
     * @param ext
     *            the extension to filter WITHOUT THE PERIOD, eg jpg not .jpg
     * @param fileDesc
     *            the description of this file type
     * @param action
     *            a string describing the action being performed (i.e., load,
     *            save)
     * @param dir
     *            the directory to show in the chooser (or null if to not be
     *            specified)
     * @param fixExt
     *            true when the selected file returned should have the extension
     *            appeneded if it is missing
     * @return the file
     */
    public static File select(String ext, String fileDesc, String action,
            String dir, boolean fixExt) {
        return select(new String[] {ext}, fileDesc, action, dir, fixExt);
    }
    
    /**
     * Open a file chooser to select a file.
     * 
     * Returns a file, or null if cancel is selected.
     * 
     * @param ext
     *            the extension to filter WITHOUT THE PERIOD, eg jpg not .jpg
     * @param fileDesc
     *            the description of this file type
     * @param action
     *            a string describing the action being performed (i.e., load,
     *            save)
     * @param dir
     *            the directory to show in the chooser (or null if to not be
     *            specified)
     * @param fixExt
     *            true when the selected file returned should have the extension
     *            appeneded if it is missing (ext[0] will be used in this case)
     * @return the file
     */
    public static File select(String[] ext, String fileDesc, String action,
            String dir, boolean fixExt) {
        ExtensionFilter tff = new ExtensionFilter(ext, fileDesc);
        JFileChooser fc = new JFileChooser();
        if (dir != null) {
            fc.setCurrentDirectory(new File(dir));
        }
        fc.setFileFilter(tff);
        int returnVal = fc.showDialog(null, action);
        if (returnVal == JFileChooser.APPROVE_OPTION) {
            File file = fc.getSelectedFile();
            if (fixExt) {
                String theExt = ExtensionFilter.getExtension(file);
                if (theExt == null) {
                    file = new File(file.getPath() + "." + ext[0]);
                }
            }
            return file;
        }
        return null;
    }

    /**
     * A file filter for selecting files of only one given extension.
     * 
     */
    public static class ExtensionFilter extends FileFilter {
        /** The extensions to accept */
        String exts[];

        /** description of the extensions */
        String desc;

        /**
         * create a file filter which accepts only the given extension 
         * @param ext the extension string without a '.'
         * @param desc description of the extension
         */
        public ExtensionFilter(String ext, String desc) {
            exts = new String[1];
            exts[0] = ext;
            this.desc = desc;
        }

        /** 
         * create a file filter which accepts only the given extensions 
         * @param exts 
         * @param desc 
         */
        public ExtensionFilter(String[] exts, String desc) {
            this.exts = exts;
            this.desc = desc;
        }

        /** 
         * Accept only files with the desired extension 
         * @param f 
         * @return true for directories and files with the correct extension
         */
        @Override
        public boolean accept(File f) {
            if (f.isDirectory()) {
                return true;
            }
            String extension = getExtension(f);
            if (extension != null) {
                for (int i = 0; i < exts.length; i++) {
                    if (extension.equals(exts[i])) {
                        return true;
                    }
                }
                return false;
            }
            return false;
        }

        /** 
         * Get the extension of a file. 
         * @param f 
         * @return the extension
         */
        static public String getExtension(File f) {
            String ext = null;
            String s = f.getName();
            int i = s.lastIndexOf('.');
            if (i > 0 && i < s.length() - 1) {
                ext = s.substring(i + 1).toLowerCase();
            }
            return ext;
        }

        /** 
         * The description of this filter 
         * @return the description
         */
        @Override
        public String getDescription() {
            return desc;
        }
    }
}
