/*
 * Created on 4-Jan-2005
 */
package mintools.swing;

/*
** Luxor - XML User Interface Language (XUL) Toolkit
** Copyright (c) 2001, 2002 by Gerald Bauer
**
** This program is free software.
**
** You may redistribute it and/or modify it under the terms of the GNU
** General Public License as published by the Free Software Foundation.
** Version 2 of the license should be included with this distribution in
** the file LICENSE, as well as License.html. If the license is not
** included with this distribution, you may find a copy at the FSF web
** site at 'www.gnu.org' or 'www.fsf.org', or you may write to the
** Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139 USA.
**
** THIS SOFTWARE IS PROVIDED AS-IS WITHOUT WARRANTY OF ANY KIND,
** NOT EVEN THE IMPLIED WARRANTY OF MERCHANTABILITY. THE AUTHOR
** OF THIS SOFTWARE, ASSUMES _NO_ RESPONSIBILITY FOR ANY
** CONSEQUENCE RESULTING FROM THE USE, MODIFICATION, OR
** REDISTRIBUTION OF THIS SOFTWARE.
**
*/

import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.Insets;

import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;
import javax.swing.border.TitledBorder;

/**
 * CollapsiblePanel taken from xul.<p>
 * Note that you should provide your content panel with a name via setName(),
 * as this will allow for the name of the collapsed panel to be shown when 
 * the panel is minimized.
 * @author kry
 */
public class CollapsiblePanel extends JPanel implements Collapsible
{

   private static final long serialVersionUID = 7529478802493432972L;
   private CollapseButton _collapseHorizontalButton;
   private CollapseButton _collapseVerticalButton;
   private boolean _collapsed = false;
   private GridBagConstraints _constraints;

   private JPanel _content;
   private JPanel _spring;
   
   private JLabel _contentlabel; 

   /**
    * Create a new collapsible panel. <p>
    * Note that you MUST provide your content panel with a name via setName(),
    * as this will allow for the name of the collapsed panel to be shown when 
    * the panel is minimized.
    * @param content
    */
   public CollapsiblePanel( JPanel content )
   {
      super();
      _content = content;
      _contentlabel = new JLabel( " " + _content.getName() );
      setLayout( new GridBagLayout() );
      _constraints = new GridBagConstraints();
      _collapseHorizontalButton = new CollapseButton( this, SwingConstants.HORIZONTAL );
      _collapseVerticalButton = new CollapseButton( this, SwingConstants.VERTICAL );
      _spring = new JPanel();
      _spring.setLayout( new GridLayout(1,1) );
      _spring.add( _contentlabel );
      _contentlabel.setHorizontalAlignment( SwingConstants.LEFT );
      _contentlabel.setVerticalAlignment( SwingConstants.TOP );
      if ( _content.getBorder() != null && _content.getBorder() instanceof TitledBorder ) {
    	  _contentlabel.setFont( ((TitledBorder)_content.getBorder()).getTitleFont() );
      }
      expand();
   }
   
   /**
     * Create a new collapsible panel with the specified name.
     * 
     * @param content the panel to be collapsiblized
     * @param name the name to be assigned to the panel
     */
    public CollapsiblePanel(JPanel content, String name)
   {
       super();
       _content = content;
       
       _content.setName(name);
       _contentlabel = new JLabel( " " + _content.getName() );
       setLayout( new GridBagLayout() );
       _constraints = new GridBagConstraints();
       _collapseHorizontalButton = new CollapseButton( this, SwingConstants.HORIZONTAL );
       _collapseVerticalButton = new CollapseButton( this, SwingConstants.VERTICAL );
       _spring = new JPanel();
       _spring.setLayout( new GridLayout(1,1) );
       _spring.add( _contentlabel );
       _contentlabel.setHorizontalAlignment( SwingConstants.LEFT );
       _contentlabel.setVerticalAlignment( SwingConstants.TOP );
       expand();
   }

   /**
    *  Tells you whether this component is currently collapsed. Useful for
    *  checking the component's status.
    *
    * @return    true if this component is collapsed, false if it is not.
    */
   public boolean isCollapsed()
   {
      return _collapsed;
   }

   /**
    *  Tells you whether this component is collapsible.
    *
    * @return    a boolean indicating this component is collapsible.
    */
   public boolean isCollapsible()
   {
      return collapsible;
   }

   /**
    *  Collapses the panel.
    */
   public void collapse()
   {
      setVisible( false );
      removeAll();

      _constraints.gridx = 0;
      _constraints.gridy = 0;
      _constraints.gridheight = 1;
      _constraints.gridwidth = 1;
      _constraints.ipadx = 0;
      _constraints.ipady = 0;
      _constraints.weightx = 0;
      _constraints.weighty = 0;
      _constraints.insets = new Insets( 0, 0, 0, 0 );
      _constraints.anchor = GridBagConstraints.NORTHWEST;
      _constraints.fill = GridBagConstraints.NONE;
      add( _collapseHorizontalButton, _constraints );      
      Dimension dim = _collapseHorizontalButton.getSize();
      _collapseHorizontalButton.setBounds( 0, 0, dim.width, dim.height );
            
      _constraints.gridx = GridBagConstraints.RELATIVE;
      _constraints.gridy = 0;
      _constraints.gridheight = GridBagConstraints.REMAINDER;
      _constraints.gridwidth = GridBagConstraints.REMAINDER;
      _constraints.weightx = 1.0;
      _constraints.weighty = 1.0;
      _constraints.anchor = GridBagConstraints.WEST;
      _constraints.fill = GridBagConstraints.BOTH;
      add( _spring, _constraints );

      _collapsed = true;
      revalidate();
      setVisible( true );
   }

   /**
    *  Uncollapses the panel.
    */
   public void expand()
   {
      setVisible( false );
      removeAll();

      _constraints.gridx = 0;
      _constraints.gridy = 0;
      _constraints.gridheight = 1;
      _constraints.gridwidth = 1;
      _constraints.ipadx = 0;
      _constraints.ipady = 0;
      _constraints.weightx = 0;
      _constraints.weighty = 0;
      _constraints.insets = new Insets( 0, 0, 0, 0 );
      _constraints.anchor = GridBagConstraints.NORTHWEST;
      _constraints.fill = GridBagConstraints.NONE;
      add( _collapseVerticalButton, _constraints );
      Dimension dim = _collapseVerticalButton.getSize();
      _collapseVerticalButton.setBounds( 0, 0, dim.width, dim.height );

      //    constraints.insets = new Insets(5,5,5,5);
      _constraints.gridx = GridBagConstraints.RELATIVE;
      _constraints.gridy = 0;
      _constraints.gridheight = GridBagConstraints.REMAINDER;
      _constraints.gridwidth = GridBagConstraints.REMAINDER;
      _constraints.weightx = 1.0;
      _constraints.weighty = 1.0;
      _constraints.anchor = GridBagConstraints.WEST;
      _constraints.fill = GridBagConstraints.BOTH;
      add( _content, _constraints );

      _collapsed = false;
      revalidate();
      setVisible( true );
   }

}

 
